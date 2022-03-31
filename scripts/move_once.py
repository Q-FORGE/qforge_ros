#!/usr/bin/env python
# ball launcher for qforge_ros package
# Subscribes to 

import rospy
from std_msgs.msg import String,Bool,Float64,Float32
from nav_msgs.msg import Odometry
from numpy import NaN, array,append,linalg
from math import pi
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

# Fetch node rate parameter
tester_rate = rospy.get_param('tester_rate',10)

# Initialize variables
setpoint_pose = PoseStamped()
current_pose = PoseStamped()

# Define zone 2 setpoint
zone2_pose = PoseStamped()
zone2_pose.pose.position.x = -2
zone2_pose.pose.position.y = 0
zone2_pose.pose.position.z = 3

def pose_callback(msg):
    # Update current pose from mavros local position
    global current_pose
    current_pose.pose = msg.pose.pose
    current_pose.header = msg.header

def state_callback(msg):
    # Fetch most recent state from commander
    global state
    state = msg

def test_position_command():

    global state
    global setpoint_pose

    # Initialize node
    rospy.init_node('tst')
    rate = rospy.Rate(tester_rate)

    # Define vehicle state and camera subscribers
    pose_sub = rospy.Subscriber('mavros/global_position/local', Odometry, pose_callback)

    # Define target waypoint publisher
    target_pub = rospy.Publisher('tracker/input_pose', PoseStamped, queue_size = 1, latch = True)

    # Initialize publishing variables
    publish_target = False
    last_msg = rospy.Time.now()

    setpoint_pose = zone2_pose
    publish_target = True

        
    if publish_target:
        target_pub.publish(setpoint_pose)
        publish_target = False
        last_msg = rospy.Time.now()

    rate.sleep()