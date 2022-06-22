#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes estimated artag location 'ar_tag_est'
# Subscribes to ar tag measurment 'ar_point'
# Subscribes to odometry '/red/odometry'

import rospy
from scripts.pathfinder import odometry_callback
from nav_msgs.msg import Odometry

global odom_pub 
odom_pub = rospy.Publisher('/hawk2/vrpn_client/estimated_odometry', Odometry, queue_size = 1)

def odometry_callback(msg):
    msg.header.frame_id = "optitrack"
    odom_pub.publish(msg)
    


def hawk2fwd():
    rospy.Subscriber('odometry', Odometry, odometry_callback, queue_size=1, buff_size=2**24)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    try:
        hawk2fwd()
    except rospy.ROSInterruptException:
        pass
