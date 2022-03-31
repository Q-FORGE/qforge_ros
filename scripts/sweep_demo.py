#!/usr/bin/env python
# move once for qforge_ros package
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
sweeper_rate = rospy.get_param('sweeper_rate',10)

# Initialize variables
setpoint_pose = PoseStamped()
current_pose = PoseStamped()

# Define zone initial point, corners, and search altitudes array (no altitude)
zone2_init = [1,0] 
corners = [[1.5,5], [9.5,5], [9.5,-5],[1.5,-5]]
altitudes = [2, 5, 8]



def pose_callback(msg):
    # Update current pose from mavros local position
    global current_pose
    current_pose.pose = msg.pose.pose
    current_pose.header = msg.header

def sweeper():
    global setpoint_pose

    # Initialize node
    rospy.init_node('sweeper')
    rate = rospy.Rate(sweeper_rate)
    sweep_status = 0
    altitude_counter = 0

    # Define vehicle state and camera subscribers
    pose_sub = rospy.Subscriber('mavros/global_position/local', Odometry, pose_callback)

    # Define target waypoint publisher
    target_pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size = 1, latch = True)

    # Initialize publishing variables
    publish_target = False
    last_msg = rospy.Time.now()

    while not rospy.is_shutdown():
        
        if sweep_status == 0:
            now = rospy.Time.now()            
            if (now.secs - last_msg.secs > 5.):
                publish_target = True 
            setpoint_pose.pose.position.x = zone2_init[0]
            setpoint_pose.pose.position.y = zone2_init[1]
            setpoint_pose.pose.position.z = altitudes[0]

        
        if publish_target:
            target_pub.publish(setpoint_pose)
            publish_target = False
            last_msg = rospy.Time.now()

        rate.sleep()


if __name__ == '__main__':
    try:
        sweeper()
    except rospy.ROSInterruptException:
        pass