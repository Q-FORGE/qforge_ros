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
    global current_pose
    global sweep_status 
    global step_counter
    global altitude_counter

    # Initialize node
    rospy.init_node('sweeper')
    rate = rospy.Rate(sweeper_rate)
    
    altitude_counter = 0
    setpoint_step = 3
    sweep_status = 0
    step_counter = 0

    # Define vehicle position subscriber
    pose_sub = rospy.Subscriber('/red/odometry', Odometry, pose_callback)

    # Define target waypoint publisher
    target_pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size = 1, latch = True)

    # Vehicle position and position error for debugging
    debug_pub = rospy.Publisher('Sweeper_debug', String, queue_size=1)


    # Initialize publishing variables
    publish_target = False
    last_msg = rospy.Time.now()


    while not rospy.is_shutdown():
        now = rospy.Time.now()        
        pose_sub = rospy.Subscriber('/red/odometry', Odometry, pose_callback)
        vehicle_position = array([[current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z]])

        if sweep_status == 0:                
            setpoint_pose.pose.position.x = zone2_init[0]
            setpoint_pose.pose.position.y = zone2_init[1]
            setpoint_pose.pose.position.z = altitudes[0]
            reference_position = array([[setpoint_pose.pose.position.x,setpoint_pose.pose.position.y,setpoint_pose.pose.position.z]])
            position_error = vehicle_position-reference_position
            error_mag = linalg.norm(position_error[0,0:3])
            if error_mag < 1:
                sweep_status = 1
            
        elif sweep_status == 1:
            setpoint_pose.pose.position.x = corners[0] [0] + step_counter*setpoint_step
            setpoint_pose.pose.position.y = corners[0] [1]
            setpoint_pose.pose.position.z = altitudes[altitude_counter]
            setpoint_pose.pose.orientation.x = 0
            setpoint_pose.pose.orientation.y = 0
            setpoint_pose.pose.orientation.z = 0.7071
            setpoint_pose.pose.orientation.w = 0.7071  

            reference_position = array([[setpoint_pose.pose.position.x,setpoint_pose.pose.position.y,setpoint_pose.pose.position.z]])
            position_error = vehicle_position-reference_position
            error_mag = linalg.norm(position_error[0,0:3])
            if error_mag < 1:
                step_counter += 1
                if setpoint_pose.pose.position.x >= corners[1] [0]:
                    setpoint_pose.pose.position.x = corners[1] [0]
                    step_counter = 0
                    sweep_status = 2


        elif sweep_status == 2:
            setpoint_pose.pose.position.x = corners[1] [0]
            setpoint_pose.pose.position.y = corners[1] [1] - step_counter*setpoint_step
            setpoint_pose.pose.position.z = altitudes[altitude_counter]
            setpoint_pose.pose.orientation.x = 0
            setpoint_pose.pose.orientation.y = 0
            setpoint_pose.pose.orientation.z = 0
            setpoint_pose.pose.orientation.w = 0  

            reference_position = array([[setpoint_pose.pose.position.x,setpoint_pose.pose.position.y,setpoint_pose.pose.position.z]])
            position_error = vehicle_position-reference_position
            error_mag = linalg.norm(position_error[0,0:3])
            if error_mag < 1:
                step_counter += 1
                if setpoint_pose.pose.position.y <= corners[2] [1]:
                    setpoint_pose.pose.position.y = corners[2] [1]
                    step_counter = 0
                    sweep_status = 3


        elif sweep_status == 3:
            setpoint_pose.pose.position.x = corners[2] [0] - step_counter*setpoint_step 
            setpoint_pose.pose.position.y = corners[2] [1]
            setpoint_pose.pose.position.z = altitudes[altitude_counter]
            setpoint_pose.pose.orientation.x = 0
            setpoint_pose.pose.orientation.y = 0
            setpoint_pose.pose.orientation.z = -0.7071
            setpoint_pose.pose.orientation.w = 0.7071  

            reference_position = array([[setpoint_pose.pose.position.x,setpoint_pose.pose.position.y,setpoint_pose.pose.position.z]])
            position_error = vehicle_position-reference_position
            error_mag = linalg.norm(position_error[0,0:3])
            if error_mag < 1:
                step_counter += 1
                if setpoint_pose.pose.position.x <= corners[3] [0]:
                    setpoint_pose.pose.position.x = corners[3] [0]
                    step_counter = 0
                    sweep_status = 4
                    altitude_counter += 1 

        elif sweep_status == 4:
            setpoint_pose.pose.position.x = corners[3] [0] + step_counter*setpoint_step 
            setpoint_pose.pose.position.y = corners[3] [1]
            setpoint_pose.pose.position.z = altitudes[altitude_counter]
            setpoint_pose.pose.orientation.x = 0
            setpoint_pose.pose.orientation.y = 0
            setpoint_pose.pose.orientation.z = -0.7071
            setpoint_pose.pose.orientation.w = 0.7071  

            reference_position = array([[setpoint_pose.pose.position.x,setpoint_pose.pose.position.y,setpoint_pose.pose.position.z]])
            position_error = vehicle_position-reference_position
            error_mag = linalg.norm(position_error[0,0:3])
            if error_mag < 1:
                step_counter += 1
                if setpoint_pose.pose.position.x >= corners[2] [0]:
                    setpoint_pose.pose.position.x = corners[2] [0]
                    step_counter = 0
                    sweep_status = 5

        elif sweep_status == 5:
            setpoint_pose.pose.position.x = corners[2] [0]
            setpoint_pose.pose.position.y = corners[2] [1] + step_counter*setpoint_step
            setpoint_pose.pose.position.z = altitudes[altitude_counter]
            setpoint_pose.pose.orientation.x = 0
            setpoint_pose.pose.orientation.y = 0
            setpoint_pose.pose.orientation.z = 0
            setpoint_pose.pose.orientation.w = 0  

            reference_position = array([[setpoint_pose.pose.position.x,setpoint_pose.pose.position.y,setpoint_pose.pose.position.z]])
            position_error = vehicle_position-reference_position
            error_mag = linalg.norm(position_error[0,0:3])
            if error_mag < 1:
                step_counter += 1
                if setpoint_pose.pose.position.y >= corners[1] [1]:
                    setpoint_pose.pose.position.y = corners[1] [1]
                    step_counter = 0
                    sweep_status = 6

        
        elif sweep_status == 6:
            setpoint_pose.pose.position.x = corners[1] [0] - step_counter*setpoint_step 
            setpoint_pose.pose.position.y = corners[1] [1]
            setpoint_pose.pose.position.z = altitudes[altitude_counter]
            setpoint_pose.pose.orientation.x = 0
            setpoint_pose.pose.orientation.y = 0
            setpoint_pose.pose.orientation.z = 0.7071
            setpoint_pose.pose.orientation.w = 0.7071  

            reference_position = array([[setpoint_pose.pose.position.x,setpoint_pose.pose.position.y,setpoint_pose.pose.position.z]])
            position_error = vehicle_position-reference_position
            error_mag = linalg.norm(position_error[0,0:3])
            if error_mag < 1:
                step_counter += 1
                if setpoint_pose.pose.position.x <= corners[0] [0]:
                    setpoint_pose.pose.position.x = corners[0] [0]
                    step_counter = 0
                    sweep_status = 7

        
        x0 = append(vehicle_position,position_error)
        dbg_string = "position = [%.2f,%.2f,%.2f], error = [%.2f,%.2f,%.2f]" %(x0[0],x0[1],x0[2],x0[3],x0[4],x0[5])
        debug_pub.publish(dbg_string)


        if (now.secs - last_msg.secs > 2.):
            publish_target = True 
            

        
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