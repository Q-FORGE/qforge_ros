#!/usr/bin/env python
# service for area 3 sweep and AR search

import rospy
import numpy as np
from math import pi, modf
from scipy.spatial.transform import Rotation as R
import std_msgs.msg
from geometry_msgs.msg import Transform,Quaternion,Twist,Vector3,PoseStamped,Pose,Point
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


