#!/usr/bin/env python
# service node to determine delivery error

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from qforge_ros.srv import LaunchSight, LaunchSightResponse

g = -9.81;
t_end = 5; # check ball trajectry in the next t_end second
response = LaunchSightResponse()

def calculate_launch_sight(req):

    uav_pos_i = np.array([[req.odometry.pose.pose.position.x,req.odometry.pose.pose.position.y,\
        req.odometry.pose.pose.position.z]])
    C_ib = R.from_quat(np.array([req.odometry.pose.pose.orientation.x,req.odometry.pose.pose.orientation.y,\
        req.odometry.pose.pose.orientation.z,req.odometry.pose.pose.orientation.w]))
    uav_vel_i = C_ib.apply(np.array([[req.odometry.twist.twist.linear.x,req.odometry.twist.twist.linear.y,\
        req.odometry.twist.twist.linear.z]]))
    uav_omega_i = C_ib.apply(np.array([[req.odometry.twist.twist.angular.x,req.odometry.twist.twist.angular.y,\
        req.odometry.twist.twist.angular.z]]))
    r_qs_i = C_ib.apply([[0,0,-0.4]])

    C_delay = R.from_rotvec(req.delay*uav_omega_i)
    pos_delay = req.delay*uav_vel_i

    ball_position = uav_pos_i + pos_delay + C_delay.apply(r_qs_i)
    ball_velocity = uav_vel_i + np.cross(uav_omega_i,r_qs_i)

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

    else:
        # does not reach wall in t_end seconds
        response.error = Vector3(np.NaN,np.NaN,np.NaN)

    return response

def launch_sight_server():
    rospy.init_node('launch_sight_server')
    s = rospy.Service('launch_sight', LaunchSight, calculate_launch_sight)
    rospy.spin()

if __name__ == "__main__":
    try:
        launch_sight_server()
    except rospy.ROSInterruptException:
        pass