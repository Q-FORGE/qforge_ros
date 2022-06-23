#!/usr/bin/env python
# service for area 3 sweep

import rospy
import numpy as np
from geometry_msgs.msg import Transform,Quaternion,Point, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from qforge_ros.srv import InitialSweep, InitialSweepResponse

def calculate_initial_sweep(req):

    # Input handling
    initial_x = req.initial_position.x
    initial_y = req.initial_position.y
    initial_z = req.initial_position.z
    y_bounds = req.y_bounds
    spacing = req.y_spacing

    # Define search corners
    point_center = [initial_x,initial_y,initial_z]
    point_right = [initial_x,y_bounds[0]+spacing,initial_z]
    point_left = [initial_x,y_bounds[1]-spacing,initial_z]

    # Initialize variables

    response = InitialSweepResponse()
    response.trajectory = MultiDOFJointTrajectory()
    response.trajectory.points = []
    angle = 45*3.14/180

    p1 = MultiDOFJointTrajectoryPoint()
    p2 = MultiDOFJointTrajectoryPoint()
    p3 = MultiDOFJointTrajectoryPoint()
    p1.transforms = [Transform(translation=Vector3(point_left[0],point_left[1],point_left[2]),rotation=Quaternion(0,0,np.sin(angle/2),np.cos(angle/2)))]
    p2.transforms = [Transform(translation=Vector3(point_right[0],point_right[1],point_right[2]),rotation=Quaternion(0,0,np.sin(-angle/2),np.cos(-angle/2)))]
    p3.transforms = [Transform(translation=Vector3(point_center[0],point_center[1],point_center[2]),rotation=Quaternion(0,0,0,1))]
    response.trajectory.points.append(p1)
    response.trajectory.points.append(p2)
    response.trajectory.points.append(p3)

    return response

def initial_sweep_server():
    rospy.init_node('initial_sweep_server')
    s = rospy.Service('initial_sweep', InitialSweep, calculate_initial_sweep)
    rospy.spin()

if __name__ == "__main__":
    try:
        initial_sweep_server()
    except rospy.ROSInterruptException:
        pass
