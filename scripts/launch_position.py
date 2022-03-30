#!/usr/bin/env python
# service node to determine start position for launch operation
# input = (target_position,wall_norm)
#   target_position = geometry_msgs/Point: target position in world frame
#   wall_norm = geometry_msgs/Point: wall normal vector, must be in the x-y plane
# output = (start_transform)
#   start_transform = geometry_msgs/Transform: position and orientation of the drone to start launch operation 

from __future__ import print_function

import rospy
from numpy import array, arccos,linalg
from scipy.spatial.transform import Rotation as R
from math import pi
from geometry_msgs.msg import Transform
from qforge_ros.srv import LaunchPosition

x_offset = 10
z_offset = 5

def calculate_launch_position(req):
    target_position = array([req.target_position.x,req.target_position.y,req.target_position.z])
    wall_normal = array([req.wall_normal.x,req.wall_normal.y,req.wall_normal.z])
    # received = "target position = [%.2f,%.2f,%.2f], wall normal vector = [%.2f,%.2f,%.2f]" \
    #     %(target_position[0],target_position[1],target_position[2],wall_normal[0],wall_normal[1],wall_normal[2])
    # print(received)
    start_transform = Transform()
    start_position = target_position + x_offset*wall_normal + z_offset*array([0,0,1])
    start_transform.translation.x = start_position[0]
    start_transform.translation.y = start_position[1]
    start_transform.translation.z = start_position[2]
    psi = arccos((-wall_normal)@array([1,0,0]).T)
    print(psi*180/pi)
    r = R.from_euler('z', [psi])
    quat = r.as_quat()
    start_transform.rotation.x = quat[0]
    start_transform.rotation.y = quat[1]
    start_transform.rotation.z = quat[2]
    start_transform.rotation.w = quat[3]
    return start_transform

def launch_position_server():
    rospy.init_node('launch_position_server')
    s = rospy.Service('launch_position', LaunchPosition, calculate_launch_position)
    rospy.spin()

if __name__ == "__main__":
    launch_position_server()