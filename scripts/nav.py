#!/usr/bin/env python

# Navigation node for qforge_ros package
# Publishes PoseStamped setpoint to tracker/input_pose
# Subscribes to String current vehicle state at 'vehicle_state'
# Subscribes to Odometry vehicle pose at 'mavros/global_position/local'

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Fetch node rate parameter
nav_rate = rospy.get_param('nav_rate',10)

# Initialize variables
setpoint_pose = PoseStamped()
current_pose = PoseStamped()
state = String()

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

def navigator():

    global state
    global setpoint_pose

    # Initialize node
    rospy.init_node('nav')
    rate = rospy.Rate(nav_rate)

    # Define vehicle state and position subscribers
    state_sub = rospy.Subscriber('vehicle_state', String, state_callback)
    pose_sub = rospy.Subscriber('mavros/global_position/local', Odometry, pose_callback)

    # Define target waypoint publisher
    target_pub = rospy.Publisher('tracker/input_pose', PoseStamped, queue_size = 1, latch = True)

    # Initialize publishing variables
    publish_target = False
    last_msg = rospy.Time.now()

    state = rospy.wait_for_message('vehicle_state', String)

    while not rospy.is_shutdown():

        if state.data == 'takeoff':
            publish_target = False
        
        elif state.data == 'takeoff_complete':
            publish_target = False
        
        elif state.data == 'trans_12':
            now = rospy.Time.now()
            if (now.secs - last_msg.secs > 5.):
                publish_target = True
            setpoint_pose = zone2_pose

        
        if publish_target:
            target_pub.publish(setpoint_pose)
            publish_target = False
            last_msg = rospy.Time.now()

        rate.sleep()

if __name__ == '__main__':
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
