#!/usr/bin/env python
# service for area 3 sweep

import rospy
from geometry_msgs.msg import Transform,Quaternion,Point, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from qforge_ros.srv import InitialSweep, InitialSweepResponse

def calculate_initial_sweep(req):

    # Input handling
    initial_x = req.initial_position.x
    initial_y = req.initial_position.y
    initial_z = req.initial_position.z
    y_bound = req.y_bounds
    spacing = req.y_spacing

    # Define search corners
    wall_ofst = wall_distance*0.7071
    point_center = [initial_x,initial_y,initial_z]
    point_right = [initial_x,y_bounds[0]+spacing,inital_z]
    point_left = [initial_x,y_bounds[1]-spacing,inital_z]
    
    # Initialize variables

    response = InitialSweepResponse()
    response.trajectory = MultiDOFJointTrajectory()
    response.trajectory.points = []


    

    p = MultiDOFJointTrajectoryPoint()
    p.transforms = [Transform(translation=Vector3(point_left[0],point_left[1],point_left[2]),rotation=Quaternion(0,0,0,1))]
    response.trajectory.points.append(p)
    p.transforms = [Transform(translation=Vector3(point_right[0],point_right[1],point_right[2]),rotation=Quaternion(0,0,0,1))]
    response.trajectory.points.append(p)
    p.transforms = [Transform(translation=Vector3(point_center[0],point_center[1],point_center[2]),rotation=Quaternion(0,0,0,1))]
    response.trajectory.points.append(p)
        
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
