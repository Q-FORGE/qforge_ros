#!/usr/bin/env python

# test node for launch operation

import rospy
from qforge_ros.msg import ArTagLocation
from qforge_ros.srv import LaunchStart,LaunchStartRequest,LaunchManeuver,LaunchManeuverRequest
from std_msgs.msg import Bool,String
from geometry_msgs.msg import Point,Vector3,Transform,Quaternion,Twist,PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

target_position = Point(12.5,-3,2)
wall_normal = Vector3(-1,0,0)

trigger = False
state = 'takeoff'

def trigger_callback(msg):
    # fetch trigger state
    global trigger
    trigger = msg.data

def xo():
    global state,trigger

    rospy.init_node('xo')

    state_pub = rospy.Publisher('vehicle_state', String, queue_size = 1,latch=True)
    pos_ctr_pub = rospy.Publisher('position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=1, latch=True)
    tag_pub = rospy.Publisher('ar_tag_est', ArTagLocation, queue_size = 1,latch=True)

    rospy.Subscriber('launch/trigger', Bool, trigger_callback)

    launch_start_handle = rospy.ServiceProxy('launch_start',LaunchStart)
    launch_maneuver_handle = rospy.ServiceProxy('launch_maneuver',LaunchManeuver)

    state_pub.publish(state)

    # climb in zone 1
    point_1 = MultiDOFJointTrajectoryPoint()
    point_1.transforms = [Transform(translation=Vector3(-10,0,10),rotation=Quaternion(0,0,0,1))]
    point_1.velocities = [Twist()]
    point_1.accelerations = [Twist()]
    pos_ctr_pub.publish(point_1)
    rospy.sleep(3)
    # fly over zone 2
    point_2 = point_1
    point_2.transforms[0].translation.x = 10
    pos_ctr_pub.publish(point_2)
    rospy.sleep(5)
    # decend to zone 3
    point_3 = point_2
    point_3.transforms[0].translation.z = 3
    pos_ctr_pub.publish(point_3)
    rospy.sleep(5)

    # publish an ar tag message
    tag = ArTagLocation()
    tag.position_best = target_position
    tag.normal = wall_normal
    tag_pub.publish(tag)

    # transit to ball_drop mode
    state = 'ball_drop'
    state_pub.publish(state)

    # go to start position
    launch_start_input = LaunchStartRequest()
    launch_start_input.target_position = target_position
    launch_start_input.wall_normal = wall_normal

    launch_start = launch_start_handle(launch_start_input)
    target_pose_pub = rospy.Publisher('tracker/input_pose', PoseStamped, queue_size = 1, latch = True)
    target_pose_pub.publish(launch_start.start_point)
    rospy.sleep(5)

    # start launch maneuver
    launch_maneuver_input = LaunchManeuverRequest()
    launch_maneuver_input.target_position = target_position
    launch_maneuver_input.wall_normal = wall_normal

    while trigger==False:
        launch_maneuver = launch_maneuver_handle(launch_maneuver_input)
        rospy.sleep(1)

    state = 'mission_complete'
    state_pub.publish(state)
    rospy.spin()

if __name__ == '__main__':
    try:
        xo()
    except rospy.ROSInterruptException:
        pass