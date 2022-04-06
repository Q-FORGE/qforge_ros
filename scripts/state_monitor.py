#!/usr/bin/env python

# State monitor node for qforge_ros package
# Publishes Int16 current zone to 'current_zone'
# Publishes Bool altitude warning at 'alt_state'
# Subscribes to Odometry vehicle position at 'mavros/global_position/local'

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

# Fetch node rate parameter
monitor_rate = rospy.get_param('monitor_rate', 5)
min_height = rospy.get_param('min_height', 2.)
max_height = rospy.get_param('max_height', 10.)

# Initialize state parameters
alt_state = False
current_zone = 1

def pose_callback(msg):
    # Update 'alt_state' when topic is published

    global alt_state
    global current_zone

    height = msg.pose.pose.position.z
    x = msg.pose.pose.position.x

    if ((height < min_height) or (height > max_height)):
        alt_state = False
    else:
        alt_state = True

def state_monitor():

    # Initialize node
    rospy.init_node('state_monitor')
    rate = rospy.Rate(monitor_rate)

    # Define vehicle altitude status and zone publishers
    alt_pub = rospy.Publisher('alt_state', Bool, queue_size = 1)
    zone_pub = rospy.Publisher('current_zone', Int16, queue_size = 1)

    # Define pose subscriber
    pose_sub = rospy.Subscriber('odometry',Odometry,pose_callback)

    while not rospy.is_shutdown():

        alt_pub.publish(alt_state)
        zone_pub.publish(current_zone)
        rate.sleep()

if __name__ == '__main__':
    try:
        state_monitor()
    except rospy.ROSInterruptException:
        pass
