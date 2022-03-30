#!/usr/bin/env python
# service node to determine start position for launch operation
# input = (target_position,wall_norm)
#   target_position = geometry_msgs/Point: target position in world frame
#   wall_norm = geometry_msgs/Point: wall normal vector
# output = (start_transform)
#   start_transform = geometry_msgs/Transform: position and orientation of the drone to start launch operation 

import rospy
from numpy import array
from geometry_msgs.msg import Transform
from qforge_ros.srv import LaunchPosition

from __future__ import print_function

def calculate_launch_position(req):
    pos = array([[req.target_position.x,req.target_position.y,req.target_position.z]])
    vec = array([[req.wall_normal.x,req.wall_normal.y,req.wall_normal.z]])
    test = "position = [%.2f,%.2f,%.2f], vector = [%.2f,%.2f,%.2f]" %(pos[0],pos[1],pos[2],vec[0],vec[1],vec[2])
    print(test)
    start_transform = Transform()
    start_transform.translation.x = 0
    start_transform.translation.y = -3
    start_transform.translation.z = 5
    return start_transform

def launch_position_server():
    rospy.init_node('launch_position_server')
    s = rospy.Service('launch_position', LaunchPosition, calculate_launch_position)
    rospy.spin()

if __name__ == "__main__":
    launch_position_server()