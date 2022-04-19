#!/usr/bin/env python
# use to test search routine service

import rospy
import std_msgs.msg
from geometry_msgs.msg import Transform,Quaternion,Twist,Vector3,PoseStamped,Pose,Point
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from qforge_ros.srv import SearchRoutine, SearchRoutineRequest



def search_routine_tester():
    rospy.wait_for_service('search_routine')
    search_routine_handle = rospy.ServiceProxy('search_routine',SearchRoutine)
    search_routine_input = SearchRoutineRequest()

    search_routine_input.x_bounds = [1,12]
    search_routine_input.y_bounds = [-6.5,6.5]
    search_routine_input.z_bounds = [2,4.5]
    search_routine_input.wall_dist = 3
    search_routine_input.vert_range = 1
    search_routine_input.ccw_flag = True
    
    search_routine = search_routine_handle(search_routine_input)

    rospy.init_node('search_routine_tester')
    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)

    initial_trajectory = MultiDOFJointTrajectory()
    initial_pose = MultiDOFJointTrajectoryPoint()
    initial_pose.transforms = [Transform(translation=Vector3(2,0,2),rotation=Quaternion(0,0,0,1))]
    initial_trajectory.points = [initial_pose]
    trajectory_pub.publish(initial_trajectory)
    rospy.sleep(5)

    trajectory_pub.publish(search_routine.trajectory)
    rospy.sleep(5)

if __name__ == '__main__':
    try:
        search_routine_tester()
    except rospy.ROSInterruptException:
        pass



# message_test = SearchRoutineResponse

