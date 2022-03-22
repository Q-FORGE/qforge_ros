#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes estimated artag location 'ar_tag_est'
# Subscribes to ar tag measurment 'ar_point'

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from geometry_msgs.msg import Point
from tf import transformations

# globals
# global Q
# global R
# global F
# global P
# global H
global rct
# global canRun

# Fetch node rate parameter
arTag_rate = rospy.get_param('arTag_rate',50)

# Initialize state parameters
# Q = np.diag(np.array([0.01, 0.01, 0.01])) # Process covariance
# R = np.diag(np.array([0.1, 0.1, 0.1])) # Measurment covariance 

# F = np.identity(3) # state transformation
# H = np.identity(3) # measurment jacobian initialization

# P = np.identity(3) # Initialize covaraince

rct = np.array([0, 0, 0]) # camera to tag position in camera frame
# x_kk = np.array([0, 0, 0]) # initial position

# canRun = True

# def callback(data):
#     rct[0] = data.x
#     rct[1] = data.y
#     rct[2] = data.z
#     canRun = True

def arTag():

    # Initialize node
    rospy.init_node('arTag')
    rate = rospy.Rate(arTag_rate)

    # Define tag estimate publisher
    tag_pub = rospy.Publisher('ar_tag_est', Point, queue_size = 1)
    # tag_meas_sub = rospy.Subscriber('ar_point', Point, callback)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform('red', 'camera', rospy.Time())

    Q = np.diag(np.array([0.01, 0.01, 0.01])) # Process covariance
    R = np.diag(np.array([0.1, 0.1, 0.1])) # Measurment covariance 

    F = np.identity(3) # state transformation
    H = np.identity(3) # measurment jacobian initialization

    P = np.identity(3) # Initialize covaraince

    rct = np.array([0, 0, 0]) # camera to tag position in camera frame
    rct_old = rct
    x_kk = np.array([0, 0, 0]) # initial position

    canRun = True

    while not rospy.is_shutdown():
        # if canRun:
        if True:
            try:
                trans = tfBuffer.lookup_transform('red', 'camera', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                # continue
            
            # print(trans.transform)
            # trans.transform

            data = rospy.wait_for_message('/ar_point', Point)
            rct[0] = data.x
            rct[1] = data.y
            rct[2] = data.z

            
            #get Cbi
            x_kkm1 = x_kk
            P_kkm1 = np.matmul(F, np.matmul(P, np.transpose(F))) + Q
            y = rct - rct_old
            S = np.matmul(H,np.matmul(P_kkm1,np.transpose(H))) + R
            K = np.matmul(P_kkm1, np.matmul(np.transpose(H), np.linalg.inv(S)))
            x = x_kkm1 + np.dot(K, y)
            P = np.matmul((np.identity(3) - np.matmul(K, H)), P_kkm1)  

            rct_old = rct
            
            msg = Point()
            msg.x = x[0]
            msg.y = x[1]
            msg.z = x[2]

            tag_pub.publish(msg)

            canRun = False
            
            
if __name__ == '__main__':
    try:
        arTag()
    except rospy.ROSInterruptException:
        pass