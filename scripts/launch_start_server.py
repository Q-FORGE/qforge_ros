#!/usr/bin/env python
# service node to determine trajectory for launch operation

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
from qforge_ros.srv import LaunchStart, LaunchStartResponse

x_offset = 2
z_offset = 1 + 0.25
t_accel = 0.7

def calculate_launch_start(req):
    target_position = np.array([req.target_position.x,req.target_position.y,req.target_position.z])
    wall_normal = np.array([req.wall_normal.x,req.wall_normal.y,req.wall_normal.z])

    psi = np.arctan2(-wall_normal[1],-wall_normal[0])
    r = R.from_euler('z', [psi])
    HDG_quat = r.as_quat()

    start_position = target_position + x_offset*wall_normal + z_offset*np.array([0,0,1])
    if target_position[2] >= 1.:
        start_position[2] = 2.25
    start_point = PoseStamped()
    start_point.pose = Pose(position=Point(start_position[0],start_position[1],start_position[2]),\
        orientation=Quaternion(HDG_quat[0,0],HDG_quat[0,1],HDG_quat[0,2],HDG_quat[0,3]))

    response = LaunchStartResponse()
    response.start_point = start_point

    return response

def launch_start_server():
    rospy.init_node('launch_start_server')
    s = rospy.Service('launch_start', LaunchStart, calculate_launch_start)
    rospy.spin()

if __name__ == "__main__":
    try:
        launch_start_server()
    except rospy.ROSInterruptException:
        pass
