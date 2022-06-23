#!/usr/bin/env python
# service node to determine trajectory for launch operation

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3,Quaternion,Transform,Twist
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from qforge_ros.srv import LaunchManeuver,LaunchManeuverResponse
from std_msgs.msg import Bool

x_offset = 2
z_offset = 1 + 0.25
t_accel = 0.9

def perform_launch_maneuver(req):
    target_position = np.array([req.target_position.x,req.target_position.y,req.target_position.z])
    wall_normal = np.array([req.wall_normal.x,req.wall_normal.y,req.wall_normal.z])

    psi = np.arctan2(-wall_normal[1],-wall_normal[0])
    r = R.from_euler('z', [psi])
    HDG_quat = r.as_quat()

    start_position = target_position + x_offset*wall_normal + z_offset*np.array([0,0,1])
    if target_position[2] >= 1.:
        start_position[2] = 2.25
    start_point = MultiDOFJointTrajectoryPoint()
    start_point.transforms = [Transform(translation=Vector3(start_position[0],start_position[1],start_position[2]),\
    rotation=Quaternion(HDG_quat[0,0],HDG_quat[0,1],HDG_quat[0,2],HDG_quat[0,3]))]
    start_point.velocities = [Twist()]
    start_point.accelerations = [Twist()]

    launch_position = target_position - 0*wall_normal
    launch_position[2] = start_position[2]
    launch_point = MultiDOFJointTrajectoryPoint()
    launch_point.transforms = [Transform(translation=Vector3(launch_position[0],launch_position[1],launch_position[2]),\
        rotation=Quaternion(HDG_quat[0,0],HDG_quat[0,1],HDG_quat[0,2],HDG_quat[0,3]))]
    launch_point.velocities = [Twist()]
    launch_point.accelerations = [Twist()]

    end_position = target_position + 0*wall_normal
    end_position[2] = start_position[2]
    end_point = MultiDOFJointTrajectoryPoint()
    end_point.transforms = [Transform(translation=Vector3(end_position[0],end_position[1],end_position[2]),\
        rotation=Quaternion(HDG_quat[0,0],HDG_quat[0,1],HDG_quat[0,2],HDG_quat[0,3]))]
    end_point.velocities = [Twist()]
    end_point.accelerations = [Twist()]

    rospy.sleep(1)
    pos_ctr_pub.publish(launch_point)
    rospy.sleep(t_accel)
    pos_ctr_pub.publish(end_point)
    rospy.sleep(t_accel)
    pos_ctr_pub.publish(start_point)
    rospy.sleep(3)

    response = LaunchManeuverResponse()
    response.success = Bool(True)

    return response

def launch_maneuver_server():
    rospy.init_node('launch_maneuver_server')
    global pos_ctr_pub
    pos_ctr_pub = rospy.Publisher('position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=1, latch=True)
    s = rospy.Service('launch_maneuver', LaunchManeuver, perform_launch_maneuver)
    rospy.spin()

if __name__ == "__main__":
    try:
        launch_maneuver_server()
    except rospy.ROSInterruptException:
        pass
