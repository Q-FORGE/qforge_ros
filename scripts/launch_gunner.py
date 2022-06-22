#!/usr/bin/env python
# runtime node to give release trigger

import rospy
import numpy as np
from geometry_msgs.msg import Point,Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,Float32,String
from qforge_ros.msg import ArTagLocation
from qforge_ros.srv import LaunchSight, LaunchSightRequest

launch_gunner_rate = rospy.get_param('launch_gunner_rate',150)

# initialize variables
state = String()
info_received = False
delay = 0.25 # seconds

def state_callback(msg):
    # fetch state from commander
    global state
    state = msg

def odometry_callback(msg):
    global ball_odometry
    ball_odometry = msg

def tag_callback(msg):
    # fetch tag location and wall normall vector from arTag
    global target_position, wall_normal
    target_position = msg.position_best
    wall_normal = msg.normal

def launch_gunner():
    # initialize variables
    global state, target_position, wall_normal, ball_odometry, info_received
    tolerance_base = 0.1;
    tolerance = tolerance_base;
    trigger = False
    launch_sight_input = LaunchSightRequest()

    # initialize node
    rospy.init_node('launch_gunner')
    rate = rospy.Rate(launch_gunner_rate)

    # define publisher
    gunner_pub = rospy.Publisher('launch/gunner_status', String, queue_size=1,latch=True)
    error_pub = rospy.Publisher('launch/error', Float32, queue_size=1)
    trigger_pub = rospy.Publisher('launch/trigger', Bool, queue_size=1,latch=True)
    magnet_pub = rospy.Publisher('uav_magnet/gain', Float32, queue_size=1,latch=True)

    # define subscriber
    rospy.Subscriber('hawk2/vrpn_client/estimated_odometry', Odometry, odometry_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('vehicle_state', String, state_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('ar_tag_est', ArTagLocation, tag_callback, queue_size=1, buff_size=2**24)

    # define service handle
    launch_sight_handle = rospy.ServiceProxy('launch_sight',LaunchSight)

    # main loop
    while not rospy.is_shutdown():
        if state.data == 'ball_drop':
            # only run main functions if the vehicle is in ball_drop mode
            gunner_pub.publish(String('Active'))

            # update target info and tolerance when first enter ball_drop mode
            if info_received == False:
                launch_sight_input.target_position = target_position
                launch_sight_input.wall_normal = wall_normal
                # update tolerance for high target
                if launch_sight_input.target_position.z >= 2.5:
                    tolerance = tolerance + (1 - tolerance)*(launch_sight_input.target_position.z-2.5)
                # update first time flag
                info_received = True
                time_start = rospy.Time.now()

            # pass vehicle info to sight and receive launch error
            launch_sight_input.odometry = ball_odometry
            launch_sight_input.delay = delay
            launch_sight = launch_sight_handle(launch_sight_input)

            # check error
            error = np.array([[launch_sight.error.x,launch_sight.error.y,launch_sight.error.z]])
            # if error is not np.NaN
            if not (error[0,0] != error[0,0]):
                time_now = rospy.Time.now()
                time_elapsed = time_now.to_sec()-time_start.to_sec()
                tolerance = tolerance_base + 0*time_elapsed
                # update tirgger flag and launch ball if the error is smaller than tolerance
                if np.sqrt(error@error.T)<tolerance:
                    trigger = Bool(True)
                    ##simulated delay##
                    rospy.sleep(delay)
                    ###################
                    magnet_pub.publish(0.0)

            # publish error and trigger flag
            error_pub.publish(Float32(np.sqrt(error@error.T)))
            trigger_pub.publish(trigger)

            rate.sleep()

        else:
            # go straight to sleep when the vehicle is in other modes
            gunner_pub.publish(String('Inactive'))
            rate.sleep()

if __name__ == '__main__':
    try:
        launch_gunner()
    except rospy.ROSInterruptException:
        pass
