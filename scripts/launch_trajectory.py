#!/usr/bin/env python
# service node to determine trajectory for launch operation

from __future__ import print_function

import rospy
import numpy as np
from math import pi, modf
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
from geometry_msgs.msg import Transform,Quaternion,Twist,Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from qforge_ros.srv import LaunchTrajectory

x_offset = 0.8
z_offset = 0.4
a = 2.75
g = -9.81

def calculate_launch_trajectory(req):
    target_position = np.array([req.target_position.x,req.target_position.y,req.target_position.z])
    wall_normal = np.array([req.wall_normal.x,req.wall_normal.y,req.wall_normal.z])

    psi = np.arctan2(-wall_normal[1],-wall_normal[0])
    r = R.from_euler('z', [psi])
    HDG_quat = r.as_quat()
    
    # target height
    z_t = target_position[2]
    # release height to achieve z_m
    z_m = -(a*z_t-8*a-4*g)/(a+g)
    # vertical velocity at release
    w_m = np.sqrt(2*a*(4-z_m))
    # max time from release to reach target height
    t_m = -(np.sqrt(2*g*(z_t-z_m)+2*a*(4-z_m))+np.sqrt(2*a*(4-z_m)))/g
    # release position to achieve max x travel with travel time of t_m 
    x_m = (a*t_m**2)/2
    # horizontal velocity at release
    u_m = np.sqrt(2*a*x_m)
    t_ready = 10
    t_accel = 5
    t_decel = max(w_m,u_m)/a
    
    release_position = (x_offset+x_m)*wall_normal+np.array([target_position[0],target_position[1],z_m+z_offset])
    release_velocity = np.array([u_m,0,w_m])

    release_point = MultiDOFJointTrajectoryPoint()
    release_point.transforms = [Transform(translation=Vector3(release_position[0],release_position[1],release_position[2]),\
        rotation=Quaternion(HDG_quat[0,0],HDG_quat[0,1],HDG_quat[0,2],HDG_quat[0,3]))]
    release_point.velocities = [Twist(linear=Vector3(release_velocity[0],release_velocity[1],release_velocity[2]),angular=Vector3(0,0,0))]
    release_point.accelerations = [Twist(linear=Vector3(a*wall_normal[0],a*wall_normal[1],-a),angular=Vector3(0,0,0))]
    release_point.time_from_start = rospy.Duration.from_sec(t_ready+t_accel)

    start_position = release_position + 4*wall_normal - 2*np.array([0,0,1])
    start_point = MultiDOFJointTrajectoryPoint()
    start_point.transforms = [Transform(translation=Vector3(start_position[0],start_position[1],start_position[2]),\
        rotation=Quaternion(HDG_quat[0,0],HDG_quat[0,1],HDG_quat[0,2],HDG_quat[0,3]))]   
    start_point.time_from_start = rospy.Duration.from_sec(t_ready)

    #end_position = release_position+np.array([-u_m**2/(2*a),0,w_m**2/(2*a)])
    end_position = release_position+np.array([0,0,w_m**2/(2*a)])+4*wall_normal
    end_point = MultiDOFJointTrajectoryPoint()
    end_point.transforms = [Transform(translation=Vector3(end_position[0],end_position[1],end_position[2]),\
        rotation=Quaternion(HDG_quat[0,0],HDG_quat[0,1],HDG_quat[0,2],HDG_quat[0,3]))]
    end_point.time_from_start = rospy.Duration.from_sec(t_ready+t_accel+t_decel)

    launch_trajectory = MultiDOFJointTrajectory()
    launch_trajectory.points = [start_point,release_point,end_point]
    
    return launch_trajectory

def launch_position_server():
    rospy.init_node('launch_position_server')
    s = rospy.Service('launch_trajectory', LaunchTrajectory, calculate_launch_trajectory)
    rospy.spin()

if __name__ == "__main__":
    launch_position_server()