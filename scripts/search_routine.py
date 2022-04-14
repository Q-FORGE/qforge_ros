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
    
    # Input handling
    fov_camera = [req.fov.x,req.fov.y,req.fov.z] 
    x_bound = [req.min_bound.x, req.max_bound.x]
    y_bound = [req.min_bound.y, req.max_bound.y]
    z_bound = [req.min_bound.z, req.max_bound.z] 
    # trans_time = req.translational_time
    # rot_time = req.rotational.time
    trans_time = 2.2
    rot_time = 0.5

    # Define max corners
    corner_1_point = Vector3(x_bound[0],y_bound[0],z_bound[0])
    corner_2_point = Vector3(x_bound[1],y_bound[0],z_bound[0])
    corner_3_point = Vector3(x_bound[1],y_bound[1],z_bound[0])
    corner_4_point = Vector3(x_bound[0],y_bound[1],z_bound[0])
    
    # First pass
    point_1 = corner_1_point + Vector3(0.5*fov_camera[1],fov_camera[0],0.5*fov_camera[2])
    point_2 = corner_2_point + Vector3(-0.5*fov_camera[1],fov_camera[0],0.5*fov_camera[2])
    point_3 = point_2
    point_4 = corner_3_point + Vector3(-fov_camera[0],-0.5*fov_camera[1],0.5*fov_camera[2])
    point_5 = point_4
    point_6 = corner_4_point + Vector3(0.5*fov_camera[1],-fov_camera[0],0.5*fov_camera[2])

    # Second pass
    point_7 = point_6 + Vector3(0,0,fov_camera[2])
    point_8 = corner_3_point + Vector3(-0.5*fov_camera[1],-fov_camera[0],1.5*fov_camera[2])
    point_9 = point_8
    point_10 = corner_2_point + Vector3(-fov_camera[0],0.5*fov_camera[1],1.5*fov_camera[2])
    point_11 = point_10
    point_12 = point_1 + Vector3(0,0,fov_camera[2])
    

    pose_1 = MultiDOFJointTrajectoryPoint()
    pose_1.transforms = [Transform(translation=point_1,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_1.time_from_start = trans_time
    pose_2 = MultiDOFJointTrajectoryPoint()
    pose_2.transforms = [Transform(translation=point_2,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_2.time_from_start = trans_time
    pose_3 = MultiDOFJointTrajectoryPoint()
    pose_3.transforms = [Transform(translation=point_3,rotation=Quaternion(0,0,0,0))]
    pose_3.time_from_start = rot_time
    pose_4 = MultiDOFJointTrajectoryPoint()
    pose_4.transforms = [Transform(translation=point_4,rotation=Quaternion(0,0,0,0))]
    pose_4.time_from_start = trans_time
    pose_5 = MultiDOFJointTrajectoryPoint()
    pose_5.transforms = [Transform(translation=point_5,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_5.time_from_start = rot_time
    pose_6 = MultiDOFJointTrajectoryPoint()
    pose_6.transforms = [Transform(translation=point_6,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_6.time_from_start = trans_time
    pose_7 = MultiDOFJointTrajectoryPoint()
    pose_7.transforms = [Transform(translation=point_7,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_7.time_from_start = trans_time
    pose_8 = MultiDOFJointTrajectoryPoint()
    pose_8.transforms = [Transform(translation=point_8,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_8.time_from_start = trans_time
    pose_9 = MultiDOFJointTrajectoryPoint()
    pose_9.transforms = [Transform(translation=point_9,rotation=Quaternion(0,0,0,0))]
    pose_9.time_from_start = rot_time
    pose_10 = MultiDOFJointTrajectoryPoint()
    pose_10.transforms = [Transform(translation=point_10,rotation=Quaternion(0,0,0,0))]
    pose_10.time_from_start = trans_time
    pose_11 = MultiDOFJointTrajectoryPoint()
    pose_11.transforms = [Transform(translation=point_11,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_11.time_from_start = rot_time
    pose_12 = MultiDOFJointTrajectoryPoint()
    pose_12.transforms = [Transform(translation=point_12,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_12.time_from_start = trans_time


    initial_pose = MultiDOFJointTrajectoryPoint()
    initial_pose.transforms = [Transform(translation=Vector3(2,0,2),rotation=Quaternion(0,0,0,1))]
    initial_pose.time_from_start = trans_time

    response = SearchRoutineResponse()
    response.trajectory = MultiDOFJointTrajectory()
    response.trajectory.points = [initial_pose, point_1,point_2,point_3,point_4,point_5,
        point_6,point_7,point_8,point_9,point_10,point_11,point_12]
    
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

