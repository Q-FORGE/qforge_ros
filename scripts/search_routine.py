#!/usr/bin/env python
# service for area 3 sweep

import rospy
from geometry_msgs.msg import Transform,Quaternion,Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from qforge_ros.srv import SearchRoutine, SearchRoutineResponse

def calculate_search_trajectory(req):

    # Input handling
    wall_distance = req.wall_dist         # camera depth (4 is nominal, less is reccommended)
    vertical_range = req.vert_range       # vertical camera range (1 gives best results)
    x_bound = req.x_bounds                # bounding boxes
    y_bound = req.y_bounds
    z_bound = req.z_bounds
    flag_direction = req.ccw_flag         # If true, first sweep is counter clockwise

    # Define search corners
    wall_ofst = wall_distance*0.7071
    corner_1 = [x_bound[0]+wall_ofst, y_bound[0]+wall_ofst]
    corner_2 = [x_bound[1]-wall_ofst, y_bound[0]+wall_ofst]
    corner_3 = [x_bound[1]-wall_ofst, y_bound[1]-wall_ofst]
    corner_4 = [x_bound[0]+wall_ofst, y_bound[1]-wall_ofst]
    center_12 = [(corner_2[0]-corner_1[0])/2+corner_1[0],corner_1[1]]
    center_34 = [(corner_3[0]-corner_4[0])/2+corner_4[0],corner_4[1]]
    center_23 = [corner_2[0], (corner_3[1]-corner_2[1])/2+corner_2[1]]


    # Initialize variables
    add_sweep_flag = True   # If true, do another ccw or cw sweep
    swp_alt = z_bound[0] + 0.5*vertical_range  # Altitude for sweep cycle

    response = SearchRoutineResponse()
    response.trajectory = MultiDOFJointTrajectory()
    response.trajectory.points = []


    # Build trajectory
    while add_sweep_flag:
        p1 = MultiDOFJointTrajectoryPoint()
        p1.transforms = [Transform(translation=Vector3(corner_1[0],corner_1[1],swp_alt),rotation=Quaternion(0,0,-0.7071,0.7071))]
        p2 = MultiDOFJointTrajectoryPoint()
        p2.transforms = [Transform(translation=Vector3(corner_2[0],corner_2[1],swp_alt),rotation=Quaternion(0,0,-0.7071,0.7071))]
        p3 = MultiDOFJointTrajectoryPoint()
        p3.transforms = [Transform(translation=Vector3(corner_2[0],corner_2[1],swp_alt),rotation=Quaternion(0,0,0,1))]
        p4 = MultiDOFJointTrajectoryPoint()
        p4.transforms = [Transform(translation=Vector3(corner_3[0],corner_3[1],swp_alt),rotation=Quaternion(0,0,0,1))]
        p5 = MultiDOFJointTrajectoryPoint()
        p5.transforms = [Transform(translation=Vector3(corner_3[0],corner_3[1],swp_alt),rotation=Quaternion(0,0,0.7071,0.7071))]
        p6 = MultiDOFJointTrajectoryPoint()
        p6.transforms = [Transform(translation=Vector3(corner_4[0],corner_4[1],swp_alt),rotation=Quaternion(0,0,0.7071,0.7071))]

        p12 = MultiDOFJointTrajectoryPoint()
        p12.transforms = [Transform(translation=Vector3(center_12[0],center_12[1],swp_alt),rotation=Quaternion(0,0,-0.7071,0.7071))]
        p23 = MultiDOFJointTrajectoryPoint()
        p23.transforms = [Transform(translation=Vector3(center_23[0],center_23[1],swp_alt),rotation=Quaternion(0,0,0,1))]
        p34 = MultiDOFJointTrajectoryPoint()
        p34.transforms = [Transform(translation=Vector3(center_34[0],center_34[1],swp_alt),rotation=Quaternion(0,0,0.7071,0.7071))]


        if flag_direction == True:
            # ccw sweep
            response.trajectory.points.append(p1)
            response.trajectory.points.append(p12)
            response.trajectory.points.append(p2)
            response.trajectory.points.append(p3)
            response.trajectory.points.append(p23)
            response.trajectory.points.append(p4)
            response.trajectory.points.append(p5)
            response.trajectory.points.append(p34)
            response.trajectory.points.append(p6)
            flag_direction = False
        else:
            # cw sweep
            response.trajectory.points.append(p6)
            response.trajectory.points.append(p34)
            response.trajectory.points.append(p5)
            response.trajectory.points.append(p4)
            response.trajectory.points.append(p23)
            response.trajectory.points.append(p3)
            response.trajectory.points.append(p2)
            response.trajectory.points.append(p12)
            response.trajectory.points.append(p1)
            flag_direction = True

        swp_alt = swp_alt + vertical_range
        if swp_alt >= z_bound[1]:
            alt_temp = swp_alt - 0.5* vertical_range
            if alt_temp < z_bound[1]:
                # Final additional sweep ensures full area coverage
                swp_alt = z_bound[1] - 0.5*vertical_range
            else:
                add_sweep_flag = False


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
