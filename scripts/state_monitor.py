#!/usr/bin/env python

# State monitor node for qforge_ros package
# Publishes Int16 current zone to 'current_zone'
# Publishes Bool altitude warning at 'alt_state'
# Publishes Bool drop_primed at 'launch/drop_primed'
# Subscribes to Odometry vehicle position at 'odometry'
# Subscribes to PoseStamped ball drop start position at 'launch/start_pose'

import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped

# Fetch parameters
monitor_rate = rospy.get_param('monitor_rate', 5)
min_height = rospy.get_param('min_height', 2.)
max_height = rospy.get_param('max_height', 10.)
drop_start_tol = rospy.get_param('drop_start_tol', 0.1)

# Initialize state parameters
alt_state = False
current_zone = 1
current_pose = Pose()

def pose_callback(msg):
    # Update 'alt_state' and 'current_pose' when topic is published

    global alt_state
    global current_zone
    global current_pose

    current_pose = msg.pose.pose
    height = msg.pose.pose.position.z
    x = msg.pose.pose.position.x

    if ((height < min_height) or (height > max_height)):
        alt_state = False
    else:
        alt_state = True

    if ((x >= -12.5) and (x < -8.)):
        current_zone = 1
    elif ((x >= -8.) and (x < 1.)):
        current_zone = 2
    elif ((x >= 1.) and (x < 12.5)):
        current_zone = 3
    else:
        current_zone = -1

def start_callback(msg, args):
    # Publish drop_primed when vehicle within start pose

    ready_pub = args[0]

    dist = np.array([msg.pose.position.x - current_pose.position.x,
        msg.pose.position.y - current_pose.position.y,
        msg.pose.position.z - current_pose.position.z])

    if (numpy.linalg.norm(dist) < drop_start_tol):
        ready_pub.publish(True)
    else:
        ready_pub.publish(False)

def state_monitor():

    # Initialize node
    rospy.init_node('state_monitor')
    rate = rospy.Rate(monitor_rate)

    # Define vehicle altitude status, drop_status, and zone publishers
    alt_pub = rospy.Publisher('alt_state', Bool, queue_size = 1)
    zone_pub = rospy.Publisher('current_zone', Int16, queue_size = 1)
    ready_pub = rospy.Publisher('launch/drop_primed', Bool, queue_size = 1)

    # Define pose subscriber
    pose_sub = rospy.Subscriber('odometry',Odometry, pose_callback)
    start_pose_sub = rospy.Subscriber('launch/start_pose', PoseStamped,
            start_callback, (ready_pub))

    while not rospy.is_shutdown():

        alt_pub.publish(alt_state)
        zone_pub.publish(current_zone)
        rate.sleep()

if __name__ == '__main__':
    try:
        state_monitor()
    except rospy.ROSInterruptException:
        pass
