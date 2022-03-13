#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes estimated artag location 'ar_tag_est'
# Subscribes to ar tag measurment 'ar_point'

import rospy
import tf
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from geometry_msgs.msg import Point
from tf import transformations

# Fetch node rate parameter
arTag_rate = rospy.get_param('arTag_rate',50)

# Initialize state parameters
Q = np.diag(np.array([0.01, 0.01, 0.01])) # Process covariance
R = np.diag(np.array([0.1, 0.1, 0.1])) # Measurment covariance 

F = np.identity(3) # state transformation
H = np.identity(3) # measurment jacobian initialization

P = np.identity(3) # Initialize covaraince

rct = np.array([0, 0, 0]) # camera to tag position in camera frame

canRun = True

def callback(data):
    rct = data.data
    canRun = True

def arTag():

    # Initialize node
    rospy.init_node('arTag')
    rate = rospy.Rate(arTag_rate)

    # Define tag estimate publisher
    tag_pub = rospy.Publisher('ar_tag_est', Point, queue_size = 1)
    tag_meas_sub = Subscriber('ar_point', Point, callback)

    while not rospy.is_shutdown():
        if canRun:
            
