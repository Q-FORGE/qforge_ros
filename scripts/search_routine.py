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
    # fov_camera = np.array([req.fov.x,req.fov.y,req.fov.z]) 
    fov_camera = np.array([4, 4, 1])
    # x_bound = [req.min_bound.x, req.max_bound.x]
    # y_bound = [req.min_bound.y, req.max_bound.y]
    # z_bound = [req.min_bound.z, req.max_bound.z] 
    # # trans_time = req.translational_time
    # # rot_time = req.rotational.time
    trans_time = 5
    rot_time = 0.5
    x_bound = req.x_bounds
    y_bound = req.y_bounds
    z_bound = req.z_bounds

    # Define max corners
    corner_1 = [x_bound[0],y_bound[0],z_bound[0]]
    corner_2 = [x_bound[1],y_bound[0],z_bound[0]]
    corner_3 = [x_bound[1],y_bound[1],z_bound[0]]
    corner_4 = [x_bound[0],y_bound[1],z_bound[0]]
    
    # # First pass
    # point_1 = Vector3(corner_1[0]+0.5*fov_camera[1],corner_1[1]+fov_camera[0],corner_1[2]+0.5*fov_camera[2])
    # point_2 = Vector3(corner_2[0]-0.5*fov_camera[1],corner_2[1]+fov_camera[0],corner_2[2]+0.5*fov_camera[2])
    # point_3 = point_2
    # point_4 = Vector3(corner_3[0]-fov_camera[0],corner_3[1]-0.5*fov_camera[1],corner_3[2]+0.5*fov_camera[2])
    # point_5 = point_4
    # point_6 = Vector3(corner_4[0]+0.5*fov_camera[1],corner_4[1]-fov_camera[0],corner_4[2]+0.5*fov_camera[2])

    # # Second pass
    # point_7 = Vector3(corner_4[0]+0.5*fov_camera[1],corner_4[1]-fov_camera[0],corner_4[2]+1.5*fov_camera[2])
    # point_8 = Vector3(corner_3[0]-0.5*fov_camera[1],corner_3[1]-fov_camera[0],corner_3[2]+1.5*fov_camera[2])
    # point_9 = point_8
    # point_10 = Vector3(corner_2[0]-fov_camera[0],corner_2[1]+0.5*fov_camera[1],corner_2[2]+1.5*fov_camera[2])
    # point_11 = point_10
    # point_12 = Vector3(corner_1[0]+0.5*fov_camera[1],corner_1[1]+fov_camera[0],corner_1[2]+1.5*fov_camera[2])
    
    # First pass alt
    offst = fov_camera[0]*0.7071
    point_1 = Vector3(corner_1[0]+offst,corner_1[1]+offst,corner_1[2]+0.5*fov_camera[2])
    point_2 = Vector3(corner_2[0]-offst,corner_2[1]+offst,corner_2[2]+0.5*fov_camera[2])
    point_3 = point_2
    point_4 = Vector3(corner_3[0]-offst,corner_3[1]-offst,corner_3[2]+0.5*fov_camera[2])
    point_5 = point_4
    point_6 = Vector3(corner_4[0]+offst,corner_4[1]-offst,corner_4[2]+0.5*fov_camera[2])

    # Second pass alt
    point_7 = Vector3(corner_4[0]+offst,corner_4[1]-offst,corner_4[2]+1.5*fov_camera[2])
    point_8 = Vector3(corner_3[0]-offst,corner_3[1]-offst,corner_3[2]+1.5*fov_camera[2])
    point_9 = point_8
    point_10 = Vector3(corner_2[0]-offst,corner_2[1]+offst,corner_2[2]+1.5*fov_camera[2])
    point_11 = point_10
    point_12 = Vector3(corner_1[0]+offst,corner_1[1]+offst,corner_1[2]+1.5*fov_camera[2])


    pose_1 = MultiDOFJointTrajectoryPoint()
    pose_1.transforms = [Transform(translation=point_1,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_1.time_from_start = rospy.Duration.from_sec(trans_time)
    pose_2 = MultiDOFJointTrajectoryPoint()
    pose_2.transforms = [Transform(translation=point_2,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_2.time_from_start = rospy.Duration.from_sec(trans_time)
    pose_3 = MultiDOFJointTrajectoryPoint()
    pose_3.transforms = [Transform(translation=point_3,rotation=Quaternion(0,0,0,0))]
    pose_3.time_from_start = rospy.Duration.from_sec(rot_time)
    pose_4 = MultiDOFJointTrajectoryPoint()
    pose_4.transforms = [Transform(translation=point_4,rotation=Quaternion(0,0,0,0))]
    pose_4.time_from_start = rospy.Duration.from_sec(trans_time)
    pose_5 = MultiDOFJointTrajectoryPoint()
    pose_5.transforms = [Transform(translation=point_5,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_5.time_from_start = rospy.Duration.from_sec(rot_time)
    pose_6 = MultiDOFJointTrajectoryPoint()
    pose_6.transforms = [Transform(translation=point_6,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_6.time_from_start = rospy.Duration.from_sec(trans_time)
    pose_7 = MultiDOFJointTrajectoryPoint()
    pose_7.transforms = [Transform(translation=point_7,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_7.time_from_start = rospy.Duration.from_sec(trans_time)
    pose_8 = MultiDOFJointTrajectoryPoint()
    pose_8.transforms = [Transform(translation=point_8,rotation=Quaternion(0,0,0.7071,0.7071))]
    pose_8.time_from_start = rospy.Duration.from_sec(trans_time)
    pose_9 = MultiDOFJointTrajectoryPoint()
    pose_9.transforms = [Transform(translation=point_9,rotation=Quaternion(0,0,0,0))]
    pose_9.time_from_start = rospy.Duration.from_sec(rot_time)
    pose_10 = MultiDOFJointTrajectoryPoint()
    pose_10.transforms = [Transform(translation=point_10,rotation=Quaternion(0,0,0,0))]
    pose_10.time_from_start = rospy.Duration.from_sec(trans_time)
    pose_11 = MultiDOFJointTrajectoryPoint()
    pose_11.transforms = [Transform(translation=point_11,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_11.time_from_start = rospy.Duration.from_sec(rot_time)
    pose_12 = MultiDOFJointTrajectoryPoint()
    pose_12.transforms = [Transform(translation=point_12,rotation=Quaternion(0,0,-0.7071,0.7071))]
    pose_12.time_from_start = rospy.Duration.from_sec(trans_time)


    initial_pose = MultiDOFJointTrajectoryPoint()
    initial_pose.transforms = [Transform(translation=Vector3(2,0,2),rotation=Quaternion(0,0,0,1))]
    initial_pose.time_from_start = rospy.Duration.from_sec(trans_time)

    response = SearchRoutineResponse()
    response.trajectory = MultiDOFJointTrajectory()
    response.trajectory.points = [initial_pose, pose_1,pose_2,pose_3,pose_4,pose_5,
        pose_6,pose_7,pose_8,pose_9,pose_10,pose_11,pose_12]
    # response.trajectory.points = [initial_pose, pose_1]

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

