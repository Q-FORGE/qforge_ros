#!/usr/bin/env python
# use to test search routine service

import rospy
import std_msgs.msg
from geometry_msgs.msg import Point,Vector3,PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from qforge_ros.srv import SearchRoutine, SearchRoutineRequest



def search_routine_tester():
    rospy.wait_for_service('search_routine')
    search_routine_handle = rospy.ServiceProxy('search_routine',SearchRoutine)
    search_routine_input = SearchRoutineRequest()
    # search_routine_input.x_bound = [2,12]
    # search_routine_input.y_bound = [-6,6]
    # search_routine_input.z_bound = [2,6]
    # search_routine_input.fov = [4,4,1]
    # search_routine_input.speed = 3.2
    search_routine_input.fov = Vector3(4,4,1)
    search_routine = search_routine_handle(search_routine_input)

    rospy.init_node('search_routine_tester')

    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)
    trajectory_pub.publish(search_routine.trajectory)
    rospy.sleep(5)

if __name__ == '__main__':
    try:
        search_routine_tester()
    except rospy.ROSInterruptException:
        pass



# message_test = SearchRoutineResponse

