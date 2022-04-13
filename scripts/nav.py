#!/usr/bin/env python

# Navigation node for qforge_ros package
# Publishes PoseStamped setpoint to 'tracker/input_pose'
# Publishes MultiDOFJointTrajectory trajectory setpoint to 'tracker/input_trajectory'
# Publishes PoseStamped ball drop start position to 'launch/start_pose'
# Subscribes to String current vehicle state at 'vehicle_state'
# Subscribes to Odometry vehicle pose at 'mavros/global_position/local'
# Calls LaunchTrajectory service at 'launch_trajectory'

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Vector3
from nav_msgs.msg import Odometry
from qforge_ros.srv import LaunchTrajectory, LaunchTrajectoryRequest
from trajectory_msgs.msg import MultiDOFJointTrajectory

# Fetch node rate parameter
nav_rate = rospy.get_param('nav_rate',10)

# Initialize variables
setpoint_pose = PoseStamped()
setpoint_traj = MultiDOFJointTrajectory()
current_pose = PoseStamped()
state = String()

# Define zone 3 setpoint
zone3_pose = PoseStamped()
zone3_pose.pose.position.x = 3
zone3_pose.pose.position.y = 0
zone3_pose.pose.position.z = 3

def pose_callback(msg):
    # Update current pose from mavros local position
    global current_pose
    current_pose.pose = msg.pose.pose
    current_pose.header = msg.header

def state_callback(msg):
    # Fetch most recent state from commander
    global state
    state = msg

def navigator():

    global state
    global setpoint_pose

    # Initialize node
    rospy.init_node('nav')
    rate = rospy.Rate(nav_rate)

    # Define vehicle state and position subscribers
    state_sub = rospy.Subscriber('vehicle_state', String, state_callback)
    pose_sub = rospy.Subscriber('odometry', Odometry, pose_callback)

    # Define target waypoint and trajectory publishers
    target_pose_pub = rospy.Publisher('tracker/input_pose', PoseStamped, queue_size = 1, latch = True)
    target_traj_pub = rospy.Publisher('tracker/input_trajectory', MultiDOFJointTrajectory, queue_size = 1, latch = True)

    # Define ball drop start position publisher
    start_pose_pub = rospy.Publisher('launch/start_pose', PoseStamped, queue_size = 1, latch = True)

    # Define launch service proxy
    launch_traj_gen_serv = rospy.ServiceProxy('launch_trajectory',LaunchTrajectory)
    launch_traj_input = LaunchTrajectoryRequest()

    # Initialize publishing variables
    publish_target = False
    publish_traj = False
    last_msg = rospy.Time.now()
    ball_traj_gen = False
    traj_started = False

    # Hardcode target position
    target_pos = Point(12.5,-3,2)
    wall_normal = Vector3(-1,0,0)

    state = rospy.wait_for_message('vehicle_state', String)

    while not rospy.is_shutdown():

        if state.data == 'takeoff':
            publish_target = False

        elif state.data == 'takeoff_complete':
            publish_target = False

        elif state.data == 'trans_12':
            now = rospy.Time.now()
            publish_traj = False
            if (now.secs - last_msg.secs > 1.):
                publish_target = True
            setpoint_pose = zone3_pose

        elif state.data == 'trans_23':
            now = rospy.Time.now()
            publish_traj = False
            if (now.secs - last_msg.secs > 1.):
                publish_target = True
            setpoint_pose = zone3_pose

        elif state.data == 'trans_to_drop':
            now = rospy.Time.now()
            publish_traj = False
            if not ball_traj_gen:
                ball_traj_gen = True
                launch_trajectory_input.target_position = target_position
                launch_trajectory_input.wall_normal = wall_normal
                launch_trajectory = launch_trajectory_gen_serv(launch_trajectory_input)
                start_pose_pub.publish(launch_trajectory.start_point)
                publish_target = True
            if (now.secs - last_msg.secs > 1.):
                publish_target = True
            setpoint_pose = launch_trajectory.start_point

        elif state.data == 'ball_drop':
            now = rospy.Time.now()
            publish_traj = True
            if not traj_started:
                publish_target = True
                traj_started = False
            else:
                publish_target = False
            setpoint_traj = launch_trajectory.trajectory

        if publish_target:
            if publish_traj:
                target_traj_pub.publish(setpoint_traj)
            else:
                target_pose_pub.publish(setpoint_pose)
            publish_target = False
            last_msg = rospy.Time.now()

        rate.sleep()

if __name__ == '__main__':
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
