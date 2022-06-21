#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes estimated artag location 'ar_tag_est'
# Subscribes to ar tag measurment 'ar_point'
# Subscribes to odometry '/red/odometry'

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from geometry_msgs.msg import PoseStamped
from qforge_ros.msg import ArTagLocation
from tf.transformations import quaternion_matrix
from tf import transformations
try:
    from cv_bridge import CvBridge
    from cv2 import imshow
    import cv2 as cv
except ImportError:
    rospy.logerr("Open CV dependency not met. Please run 'pip install opencv-python' or rebuild the image")

# Fetch node rate parameter
arTag_rate = rospy.get_param('arTag_rate',50)
Pmin = rospy.get_param('arTag_lock_lim',12e-3)
# Pmin = rospy.get_param('arTag_lock_lim',14e-10)

global zone_num
zone_num = 1

global ar_refine_flag
ar_refine_flag = False

global time_ar_refine
time_ar_refine = 0

global snappy_snap_snap 
snappy_snap_snap = False


class quat:
    def __init__(self,q):
        self.q = q
        self.R = np.identity(3)
    def calc_rot(self):
        qr = self.q[0]
        qi = self.q[1]
        qj = self.q[2]
        qk = self.q[3]

        s = np.linalg.norm(self.q)**(-2)

        self.R[0,0] = 1 - 2*s*(qj**2 + qk**2)
        self.R[0,1] = 2*s*(qi*qj-qk*qr)
        self.R[0,2] = 2*s*(qi*qk + qj*qr)
        self.R[1,0] = 2*s*(qi*qj + qk*qr)
        self.R[1,1] = 1 - 2*s*(qi**2 + qk**2)
        self.R[1,2] = 2*s*(qj*qk - qi*qr)
        self.R[2,0] = 2*s*(qi*qk - qj*qr)
        self.R[2,1] = 2*s*(qj*qk + qi*qr)
        self.R[2,2] = 1 - 2*s*(qi**2 + qj**2)

def getNorm(xt,yt):
    x1 = 12.5
    y1 = -7.5

    x2 = 12.5
    y2 = 7.5

    x3 = 1.0
    y3 = 7.5

    x4 = 1.0
    y4 = -7.5

    def lineDist(xs,ys,xe,ye,xi,yi):
        return abs((xe-xs)*(ys-yi) - (xs-xi)*(ye-ys)) / np.sqrt((xe-xs)**2 + (ye-ys)**2)

    # d1 = lineDist(x1,y1,x2,x2,xt,yt)
    # d2 = lineDist(x2,y2,x3,x3,xt,yt)
    # d3 = lineDist(x3,y3,x4,x4,xt,yt)
    # d4 = lineDist(x4,y4,x1,x1,xt,yt)

    d1 = abs(x1-xt)
    d2 = abs(y2-yt)
    d3 = abs(x4-xt)
    d4 = abs(y1-yt)

    # rospy.logwarn(" d1: " + str(d1) + " d2: " + str(d2) + " d3: " + str(d3) + " d4: " + str(d4))

    if (d1 <= d2) and (d1 <= d3) and (d1 <= d4):
        return [-1,0,0]
    elif (d2 <= d1) and (d2 <= d3) and (d2 <= d4):
        return [0,-1,0]
    elif (d3 <= d1) and (d3 <= d2) and (d3 <= d4):
        return [1,0,0]
    elif (d4 <= d1) and (d4 <= d2) and (d4 <= d3):
        return [0,1,0]

def writeBox(image,pos):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')
    phi = 0.55*(640/(2*np.tan((np.pi/180)*25)))
    width = 640
    height = 480
    tagsize = 0.2

    xMf = pos.x
    yMf = -pos.y
    zMf = pos.z

    xL = int(phi*((xMf+np.sqrt(2)*tagsize)/zMf) + 0.5*width)
    yL = -int(phi*((yMf+np.sqrt(2)*tagsize)/zMf) - 0.5*height)

    xR = int(phi*((xMf-np.sqrt(2)*tagsize)/zMf) + 0.5*width)
    yR = -int(phi*((yMf-np.sqrt(2)*tagsize)/zMf) - 0.5*height)

    cv.rectangle(cv_image,(xL,yL),(xR,yR),(0,255,0),5)
    # cv.imshow("Image window", cv_image)
    # cv.waitKey(3)
    return bridge.cv2_to_imgmsg(cv_image, "bgr8")

def current_zone_callback(msg):
    global zone_num
    zone_num = msg.data

def vehicle_state_callback(msg):
    global v_state
    global ar_refine_flag
    global time_ar_refine
    v_state = msg.data
    if (not ar_refine_flag) and (v_state == "ar_refine"):
        ar_refine_flag = True
        time_ar_refine = (rospy.Time.now()).secs

def arTag():
    # Initialize node
    rospy.init_node('ar_tag_estimator')
    rate = rospy.Rate(arTag_rate)

    # Define tag estimate publisher
    tag_pub = rospy.Publisher('ar_tag_est', ArTagLocation, queue_size = 1)
    image_pub = rospy.Publisher('tag_image_annotated', Image, queue_size = 1)
    final_pos_pub = rospy.Publisher('tag_position_reconstructed', Point, queue_size = 1)

    Q = np.diag(np.array([0.001, 0.001, 0.001])) # Process covariance
    R = np.diag(np.array([0.2, 0.2, 0.2])) # Measurment covariance

    F = np.identity(3) # state transformation
    H = np.identity(3) # measurment jacobian initialization

    P = 10*np.identity(3) # Initialize covaraince

    rtc = np.array([0.0, 0.0, 0.0]) # camera to tag position in camera frame
    rtc_old = rtc
    q = np.array([0.0, 0.0, 0.0, 0.0])
    qcb = np.array([0.500,-0.500, 0.500, -0.500])
    quat_cb = quat(qcb)
    quat_cb.calc_rot()
    Ccb = quat_cb.R
    rbi = np.array([0.0,0.0,0.0])
    rcb = np.array([0.200, 0.000, 0.050])
    x_kk = np.array([0, 0,0]) # initial position

    msg = ArTagLocation()
    snap = Image()
    tagDetect = False
    imageSnapped = False

    badness_old = 20

    tag_hits = 0


    rospy.Subscriber('current_zone', Int16, current_zone_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('vehicle_state', String, vehicle_state_callback, queue_size=1, buff_size=2**24)

    rospy.loginfo("AR tag estimator started")
    while not rospy.is_shutdown():
        data = rospy.wait_for_message('ar_point', Point)
        camPos = data
        tag_hits = tag_hits + 1
        if zone_num == 3 and tag_hits > 1:
            tagDetect = True
        else:
            tagDetect = False
        rtc[0] = data.x
        rtc[1] = data.y
        rtc[2] = data.z

        data2p = rospy.wait_for_message('odometry', Odometry)
        data2 = data2p.pose

        q[0] = data2.pose.orientation.w
        q[1] = data2.pose.orientation.x
        q[2] = data2.pose.orientation.y
        q[3] = data2.pose.orientation.z
        q_cls = quat(q) 
        q_cls.calc_rot()
        Cbi = q_cls.R
        H = np.transpose(np.matmul(Cbi,Ccb))

        rbi[0] = data2.pose.position.x
        rbi[1] = data2.pose.position.y
        rbi[2] = data2.pose.position.z

        x_kkm1 = x_kk
        P_kkm1 = np.matmul(F, np.matmul(P, np.transpose(F))) + Q
        rtc_km1 =  np.dot(np.transpose(np.matmul(Cbi,Ccb)), x_kkm1 - rbi - np.dot(np.transpose(Cbi), rcb))
        y =  (rtc  - rtc_km1)
        S = np.matmul(H,np.matmul(P_kkm1,np.transpose(H))) + R
        K = np.matmul(P_kkm1, np.matmul(np.transpose(H), np.linalg.inv(S)))
        x_kk = x_kkm1 + np.dot(K, y)
        P = np.matmul((np.identity(3) - np.matmul(K, H)), P_kkm1)
        badness = np.amax(np.diag(P))

        msg.position.x = x_kk[0]
        msg.position.y = x_kk[1]
        msg.position.z = x_kk[2]

        if badness < badness_old:
            msg.position_best.x = msg.position.x
            msg.position_best.y = msg.position.y
            msg.position_best.z = msg.position.z

            elhc = np.sqrt((msg.position.x - 12.5)**2 + (msg.position.y - 7.5)**2)
            erhc = np.sqrt((msg.position.x - 12.5)**2 + (msg.position.y + 7.5)**2)
            e_corner_limit = 1

            x1 = 12.5
            y1 = -7.5

            y2 = 7.5

            x4 = 1.0

            d1 = abs(x1-msg.position.x)
            d2 = abs(y2-msg.position.y)
            d3 = abs(x4-msg.position.x)
            d4 = abs(y1-msg.position.y)

            if snappy_snap_snap:
                if elhc > e_corner_limit and erhc > e_corner_limit:
                    if (d1 <= d2) and (d1 <= d3) and (d1 <= d4):
                        msg.position_best.x = 12.5
                    elif (d2 <= d1) and (d2 <= d3) and (d2 <= d4):
                        msg.position_best.y = 7.5
                    elif (d3 <= d1) and (d3 <= d2) and (d3 <= d4):
                        msg.position_best.x = 1
                    elif (d4 <= d1) and (d4 <= d2) and (d4 <= d3):
                        msg.position_best.y = -7.5
                

            badness_old = badness

        norm = getNorm(x_kk[0],x_kk[1])
        msg.normal.x = norm[0]
        msg.normal.y = norm[1]
        msg.normal.z = norm[2]

        wait_lim = 6 # max time to wait for tag
        outtaTime = ar_refine_flag and ((rospy.Time.now()).secs - time_ar_refine > wait_lim)

        if badness < Pmin or outtaTime:
            msg.lock = True
            if not imageSnapped:
                snap = rospy.wait_for_message('camera/color/image_raw',Image)
                image_pub.publish(writeBox(snap,camPos))
                final_pos_pub.publish(msg.position_best)
                # image_pub.publish(snap)
                imageSnapped = True
        else:
            msg.lock = False
    
        msg.detect = tagDetect
        # rospy.logwarn(badness)
        tag_pub.publish(msg)


if __name__ == '__main__':
    try:
        arTag()
    except rospy.ROSInterruptException:
        pass
