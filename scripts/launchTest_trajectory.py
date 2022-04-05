#!/usr/bin/env python
# use to test ball launch trajectory

import rospy
from geometry_msgs.msg import Point,Vector3,PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from qforge_ros.srv import LaunchTrajectory, LaunchTrajectoryRequest


def launchTest_trajectory():
    rospy.wait_for_service('launch_trajectory')
    launch_trajectory_handle = rospy.ServiceProxy('launch_trajectory',LaunchTrajectory)
    launch_trajectory_input = LaunchTrajectoryRequest()
    launch_trajectory_input.target_position = Point(12.5,-3,2)
    launch_trajectory_input.wall_normal = Vector3(-1,0,0)
    launch_trajectory = launch_trajectory_handle(launch_trajectory_input)
    
    rospy.init_node('launchTest_trajectory')

    start_point_pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=1, latch=True)
    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)
    #print(launch_trajectory.start_point)
    start_point_pub.publish(launch_trajectory.start_point)
    rospy.sleep(5)
    #print(launch_trajectory.trajectory)
    trajectory_pub.publish(launch_trajectory.trajectory)
    rospy.sleep(5)

if __name__ == '__main__':
    try:
        launchTest_trajectory()
    except rospy.ROSInterruptException:
        pass

