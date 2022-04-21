#!/usr/bin/env python
# use to test ball launch trigger

import rospy
from geometry_msgs.msg import Point,Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from qforge_ros.srv import LaunchTrigger, LaunchTriggerRequest

launch_trigger_rate = rospy.get_param('launch_trigger_rate',10)
#ball_odometry = Odometry()

def odometry_callback(msg):
    global ball_odometry
    ball_odometry = msg

def launchTest_trigger():
    global ball_odometry

    rospy.wait_for_service('launch_trigger')
    launch_trigger_handle = rospy.ServiceProxy('launch_trigger',LaunchTrigger)
    launch_trigger_input = LaunchTriggerRequest()
    launch_trigger_input.target_position = Point(12.5,-3,2)
    launch_trigger_input.wall_normal = Vector3(-1,0,0)

    rospy.init_node('launchTest_trigger')
    rate = rospy.Rate(launch_trigger_rate)
    error_pub = rospy.Publisher('launch_trigger_error', Vector3, queue_size=1)
    trigger_pub = rospy.Publisher('launch_trigger_trigger', Bool, queue_size=1)

    ball_odometry = rospy.wait_for_message('/red/ball/odometry', Odometry)

    while not rospy.is_shutdown():
        rospy.Subscriber('/red/ball/odometry', Odometry, odometry_callback, queue_size=1, buff_size=2**24)
        
        launch_trigger_input.odometry = ball_odometry
        launch_trigger = launch_trigger_handle(launch_trigger_input)

        error_pub.publish(launch_trigger.error)
        trigger_pub.publish(launch_trigger.trigger)
        rate.sleep()


if __name__ == '__main__':
    try:
        launchTest_trigger()
    except rospy.ROSInterruptException:
        pass
