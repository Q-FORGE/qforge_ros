#!/usr/bin/env python
# search test for qforge_ros package
# Subscribes to 

import rospy
from std_msgs.msg import String,Bool,Float64,Float32
from nav_msgs.msg import Odometry
from numpy import NaN, array,append,linalg
from math import pi
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint

# Fetch node rate parameter
searcher_rate = rospy.get_param('searcher_rate',10)

# Define pose contructors
zone2_init = [4,0,8] 

def searcher_test():
    global start_pose

    # Initialize node
    rospy.init_node('searcher_test')
    rate = rospy.Rate(searcher_rate)

    # Define target waypoint publisher
    target_pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=1, latch=True)
    # trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)

    # Initialize publishing variables
    # publish_target = False
    # last_msg = rospy.Time.now()
    start_pose = PoseStamped()
    rate = rospy.Rate(searcher_rate)

    # start_pose.pose.position.x = zone2_init[0]
    # start_pose.pose.position.y = zone2_init[1]
    # start_pose.pose.position.z = zone2_init[2]
    start_pose.pose.position.x = 2
    start_pose.pose.position.y = 0
    start_pose.pose.position.z = 2

    start_pose.pose.orientation.x = 0
    start_pose.pose.orientation.y = 0
    start_pose.pose.orientation.z = 0
    start_pose.pose.orientation.w = 1  

    # while not rospy.is_shutdown():
    #     now = rospy.Time.now()
        
        
        # if (now.secs - last_msg.secs > 2.):
        #         publish_target = True 

        # if publish_target:
    target_pub.publish(start_pose)
    rospy.sleep(5)
    
    
        #         publish_target = False
        #         last_msg = rospy.Time.now()

        # rate.sleep()


if __name__ == '__main__':
    try:
        searcher_test()
    except rospy.ROSInterruptException:
        pass