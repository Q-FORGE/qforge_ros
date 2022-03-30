#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes estimated artag location 'ar_tag_est'
# Subscrbies to ar tag measurment 'ar_point'

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from qforge_ros.msg import ar_tag_location
from tf.transformations import quaternion_matrix
from tf import transformations

# Fetch node rate parameter
arTag_rate = rospy.get_param('arTag_rate',50)
Pmin = rospy.get_param('arTag_lock_lim',1e-1)

class quat:
    def __init__(self,q):
        self.q = q
        self.R = np.identity(3)
    def calc_rot(self):
        qr = self.q[0]
        qi = self.q[1]
        qj = self.q[2]
        qk = self.q[3]

        self.R[0,0] = 1 - 2*(qj**2 + qk**2)
        self.R[0,1] = 2*(qi*qj-qk*qr)
        self.R[0,2] = 2*(qi*qk + qj*qr)
        self.R[1,0] = 2*(qi*qj + qk*qr)
        self.R[1,1] = 1 - 2*(qi**2 + qk**2)
        self.R[1,2] = 2*(qj*qk - qi*qr)
        self.R[2,0] = 2*(qi*qk - qj*qr)
        self.R[2,1] = 2*(qj*qk + qi*qr)
        self.R[2,2] = 1 - 2*(qi**2 + qj**2)


def arTag():

    # Initialize node
    rospy.init_node('arTag')
    rate = rospy.Rate(arTag_rate)

    # Define tag estimate publisher
    tag_pub = rospy.Publisher('ar_tag_est', ar_tag_location, queue_size = 1)

    Q = np.diag(np.array([0.01, 0.01, 0.01])) # Process covariance
    R = np.diag(np.array([0.4, 0.4, 0.4])) # Measurment covariance 

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


    while not rospy.is_shutdown():
        data = rospy.wait_for_message('/ar_point', Point)
        rtc[0] = data.x
        rtc[1] = data.y
        rtc[2] = data.z

        data2p = rospy.wait_for_message('/red/odometry', Odometry)
        data2 = data2p.pose
        
        q[0] = data2.pose.orientation.w
        q[1] = data2.pose.orientation.x
        q[2] = data2.pose.orientation.y
        q[3] = data2.pose.orientation.z
        q_cls = quat(q)
        q_cls.calc_rot()
        Cbi = q_cls.R
        H = np.transpose(np.matmul(Ccb,Cbi))

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
        print(P)
        
        msg = ar_tag_location()
        msg.position.x = x_kk[0]
        msg.position.y = x_kk[1]
        msg.position.z = x_kk[2]
        msg.normal.x = 1
        msg.normal.y = 0
        msg.normal.z = 0

        if np.linalg.det(P) < Pmin:
            msg.lock = True
        else:
            msg.lock = False

        tag_pub.publish(msg)
            
            
if __name__ == '__main__':
    try:
        arTag()
    except rospy.ROSInterruptException:
        pass