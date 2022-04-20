#!/usr/bin/env python
# use to test search routine service

import rospy
from geometry_msgs.msg import Transform,Quaternion,Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from qforge_ros.srv import SearchRoutine, SearchRoutineRequest



def kobe():
    rospy.init_node('kobe')
    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)

    wall_rush = MultiDOFJointTrajectory()
    p1 = MultiDOFJointTrajectoryPoint()
    p1.transforms = [Transform(translation=Vector3(14,-3,2),rotation=Quaternion(0,0,0,1))]
    wall_rush.points = [p1]
    trajectory_pub.publish(wall_rush)
    rospy.sleep(2)
      
    back_up = MultiDOFJointTrajectoryPoint()
    p2.transforms = [Transform(translation=Vector3(9,-3,2.5),rotation=Quaternion(0,0,0,1))]
    back_up.points = [p2]
    trajectory_pub.publish(back_up)
    rospy.sleep(5)


if __name__ == '__main__':
    try:
        kobe()
    except rospy.ROSInterruptException:
        pass
