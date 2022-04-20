#!/usr/bin/env python
# use to test search routine service

import rospy
from geometry_msgs.msg import Transform,Quaternion,Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from qforge_ros.srv import SearchRoutine, SearchRoutineRequest



def search_routine_tester():
    rospy.init_node('search_routine_tester')
    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)

    initial_trajectory = MultiDOFJointTrajectory()
    p1 = MultiDOFJointTrajectoryPoint()
    p1.transforms = [Transform(translation=Vector3(0,0,8),rotation=Quaternion(0,0,0,1))]
    p2 = MultiDOFJointTrajectoryPoint()
    wall_dist = 4
    p2.transforms = [Transform(translation=Vector3(12.5-wall_dist,-3,2),rotation=Quaternion(0,0,0,1))]
    initial_trajectory.points = [p1,p2]
    trajectory_pub.publish(initial_trajectory)
    rospy.sleep(5)


if __name__ == '__main__':
    try:
        search_routine_tester()
    except rospy.ROSInterruptException:
        pass



# message_test = SearchRoutineResponse

