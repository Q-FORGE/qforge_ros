#!/usr/bin/env python
# service for area 3 sweep 

import rospy
import numpy as np
from math import pi, modf
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
from geometry_msgs.msg import Transform,Quaternion,Twist,Vector3,PoseStamped,Pose,Point
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from qforge_ros.srv import SearchRoutine, SearchRoutineResponse

def calculate_search_trajectory(req):
    
    speed_read = req.speed

    initial_pose = MultiDOFJointTrajectoryPoint()
    initial_pose.transforms = [Transform(translation=Vector3(2,0,2),rotation=Quaternion(0,0,0,1))]
    # initial_pose.time_from_start = 2.2

    corner_1_pose = MultiDOFJointTrajectoryPoint()
    corner_1_pose.transforms = [Transform(translation=Vector3(4,2,8),rotation=Quaternion(0,0,0,1))]
    # corner_1_pose.time_from_start = 2.2

    # search_trajectory = MultiDOFJointTrajectory()
    # search_trajectory.points = [initial_pose, corner_1_pose]

    response = SearchRoutineResponse()
    response.trajectory = MultiDOFJointTrajectory()
    response.trajectory.points = [initial_pose, corner_1_pose]
    
    return response

def search_routine_server():
    rospy.init_node('search_routine_server')
    s = rospy.Service('search_routine', SearchRoutine, calculate_search_trajectory)
    rospy.spin()

if __name__ == "__main__":
    try:
        search_routine_server()
    except rospy.ROSInterruptException:
        pass

