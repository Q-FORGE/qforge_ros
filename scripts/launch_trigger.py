#!/usr/bin/env python
# service node to determine delivery error and give release trigger

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3
from qforge_ros.srv import LaunchTrigger, LaunchTriggerResponse

g = -9.81;
t_end = 5; # check ball trajectry in the next t_end second
lon_tolerance = 0.2;
lat_tolerance = 0.2;
response = LaunchTriggerResponse()

def calculate_launch_trigger(req):
    ball_position = np.array([[req.odometry.pose.pose.position.x,req.odometry.pose.pose.position.y,\
        req.odometry.pose.pose.position.z]])
    r = R.from_quat([req.odometry.pose.pose.orientation.x,req.odometry.pose.pose.orientation.y,\
        req.odometry.pose.pose.orientation.z,req.odometry.pose.pose.orientation.w])
    ball_velocity = r.apply([[req.odometry.twist.twist.linear.x,req.odometry.twist.twist.linear.y,\
        req.odometry.twist.twist.linear.z]])
    
    target_position = np.array([[req.target_position.x,req.target_position.y,req.target_position.z]])
    wall_normal = np.array([[req.wall_normal.x,req.wall_normal.y,req.wall_normal.z]])

    ball_position_end  = ball_position + ball_velocity*t_end + np.array([[0,0,1]])*(0.5*g*t_end**2)

    if ((ball_position_end-target_position)@wall_normal.T)<=0:
        # end point on the other side of the wall
        # calculation impact point
        t_impact = ((ball_position-target_position)@wall_normal.T)/(-ball_velocity@wall_normal.T)
        ball_position_impact = ball_position + ball_velocity*t_impact +\
            np.array([[0,0,1]])*(0.5*g*t_impact**2)
        delivery_error = ball_position_impact - target_position

        response.error = Vector3(delivery_error[0,0],delivery_error[0,1],delivery_error[0,2])
        if abs(delivery_error@np.array([[0,0,1]]).T)<lon_tolerance:
            response.trigger = True
        else:
            response.trigger = False
    else:
        # does not reach wall in t_end seconds
        response.error = Vector3(np.NaN,np.NaN,np.NaN)
        response.trigger = False

    return response

def launch_trigger_server():
    rospy.init_node('launch_trigger_server')
    s = rospy.Service('launch_trigger', LaunchTrigger, calculate_launch_trigger)
    rospy.spin()

if __name__ == "__main__":
    try:
        launch_trigger_server()
    except rospy.ROSInterruptException:
        pass