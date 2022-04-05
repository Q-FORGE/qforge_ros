#!/usr/bin/env python
# service node to determine delivery error and give release trigger

import rospy
import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform,Quaternion,Twist,Vector3,PoseStamped,Pose,Point
from qforge_ros.srv import LaunchTrigger, LaunchTriggerResponse

ball_position = np.array([[0,0,0]])
ball_velocity = np.array([[0,0,0]])


def odometry_callback(msg):
    global ball_position,ball_velocity
    ball_position = np.array([[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]])
    r = R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    ball_velocity = r.apply([[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]])


def calculate_launch_trigger(req):
    target_position = np.array([req.target_position.x,req.target_position.y,req.target_position.z])
    wall_normal = np.array([req.wall_normal.x,req.wall_normal.y,req.wall_normal.z])

def launch_trigger_server():
    rospy.init_node('launch_trigger_server')