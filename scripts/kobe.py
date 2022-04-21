#!/usr/bin/env python
# use to test search routine service

import rospy
from geometry_msgs.msg import Transform,Quaternion,Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from std_msgs.msg import Float32
from qforge_ros.srv import SearchRoutine, SearchRoutineRequest



def kobe():
    rospy.init_node('kobe')
    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)
    magnet_pub = rospy.Publisher('/red/uav_magnet/gain', Float32, queue_size=1, latch=True)

    initial_pose = MultiDOFJointTrajectory()
    temp_point = MultiDOFJointTrajectoryPoint()
    lob_alt = 4
    temp_point.transforms = [Transform(translation=Vector3(7.58,-3,lob_alt),rotation=Quaternion(0,0,0,1))]
    # temp_point.transforms = [Transform(translation=Vector3(8.43,-3,3),rotation=Quaternion(0,0,0,1))]
    initial_pose.points = [temp_point]
    trajectory_pub.publish(initial_pose)
    rospy.sleep(5)
    
    wall_rush = MultiDOFJointTrajectory()
    p1 = MultiDOFJointTrajectoryPoint()
    p1.transforms = [Transform(translation=Vector3(16,-3,lob_alt),rotation=Quaternion(0,0,0,1))]
    wall_rush.points = [p1]
    trajectory_pub.publish(wall_rush)
    # rospy.sleep(2)
    rospy.sleep(2.23)  

    back_up = MultiDOFJointTrajectory()
    p2 = MultiDOFJointTrajectoryPoint()
    p2.transforms = [Transform(translation=Vector3(8,-3,lob_alt),rotation=Quaternion(0,0,0,1))]
    # p3 = MultiDOFJointTrajectoryPoint()
    # p3.transforms = [Transform(translation=Vector3(8,-3,3),rotation=Quaternion(0,0,0,1))]
    back_up.points = [p2]
    trajectory_pub.publish(back_up)
    rospy.sleep(0.95)

    magnet_pub.publish(0.0)
    rospy.sleep(5)



if __name__ == '__main__':
    try:
        kobe()
    except rospy.ROSInterruptException:
        pass
