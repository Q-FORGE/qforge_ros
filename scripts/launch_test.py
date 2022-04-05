#!/usr/bin/env python
# use to test ball launch operation

import rospy
import sys
from geometry_msgs.msg import Point,Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory
from qforge_ros.srv import LaunchTrajectory


def launch():
    rospy.wait_for_service('launch_trajectory')
    launch_trajectory_handle = rospy.ServiceProxy('launch_trajectory',LaunchTrajectory)
    launch_trajectory = launch_trajectory_handle(Point(12.5,-3,2),Vector3(-1,0,0))
    
    rospy.init_node('launch_test')

    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)
    #print(launch_trajectory.trajectory)
    trajectory_pub.publish(launch_trajectory.trajectory)
    rospy.sleep(20)

if __name__ == '__main__':
    try:
        launch()
    except rospy.ROSInterruptException:
        pass

