#!/usr/bin/env python

# Navigation node for qforge_ros package
# Publishes PoseStamped setpoint to 'tracker/input_pose'
# Publishes MultiDOFJointTrajectory trajectory setpoint to 'tracker/input_trajectory'
# Publishes PoseStamped ball drop start position to 'launch/start_pose'
# Subscribes to String current vehicle state at 'vehicle_state'
# Subscribes to Odometry vehicle pose at 'mavros/global_position/local'
# Subscribes to ArTagLocation tag estimate at 'ar_tag_est'
# Subscribes to PoseStamped planner target pose at 'local_planner_manager/output_pose'
# Calls LaunchStart service at 'launch_start'
# Calls LaunchManeuver service at 'launch_maneuver'
# Calls SearchRoutine service at 'search_routine'

import rospy
from math import isclose
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion, Twist, Transform
from nav_msgs.msg import Odometry
from qforge_ros.srv import LaunchStart, LaunchStartRequest, \
        LaunchManeuver, LaunchManeuverRequest, \
        SearchRoutine, SearchRoutineRequest
from qforge_ros.msg import ArTagLocation
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

# Fetch node rate parameter
nav_rate = rospy.get_param('nav_rate',10)
refine_spacing = rospy.get_param('refine_spacing',2)

# Initialize variables
setpoint_pose = PoseStamped()
setpoint_traj = MultiDOFJointTrajectory()
setpoint_point = MultiDOFJointTrajectoryPoint()
setpoint_point.velocities = [Twist()]
setpoint_point.accelerations = [Twist()]
current_pose = PoseStamped()
state = String()
target_position = Point()
wall_normal = Vector3()
planner_pose = PoseStamped()

def pose_callback(msg):
    # Update current pose from mavros local position
    global current_pose
    current_pose.pose = msg.pose.pose
    current_pose.header = msg.header

def state_callback(msg):
    # Fetch most recent state from commander
    global state
    state = msg

def planner_callback(msg):
    # Update local planner pose setpoint
    global planner_pose
    planner_pose = msg

def tag_callback(msg):
    # Update best tag position and normal vector
    global target_position
    global wall_normal
    target_position = msg.position_best
    wall_normal = msg.normal

def quat_from_normal(normal):
    # Return quaternion facing normal vector
    if isclose(normal.y, 1., rel_tol = 1e-4):
        quat = Quaternion(0.,0.,-0.7071,0.7071)
    elif isclose(normal.y, -1., rel_tol = 1e-4):
        quat = Quaternion(0.,0.,0.7071,0.7071)
    else:
        quat = Quaternion(0.,0.,0.,1.)
    return quat

def navigator():

    global state
    global setpoint_pose

    # Initialize node
    rospy.init_node('nav')
    rate = rospy.Rate(nav_rate)

    # Define vehicle state and position subscribers
    state_sub = rospy.Subscriber('vehicle_state', String, state_callback)
    pose_sub = rospy.Subscriber('odometry', Odometry, pose_callback)
    planner_pose_sub = rospy.Subscriber('local_planner_manager/output_pose', PoseStamped, planner_callback)

    # Define ar tag location subscriber
    tag_sub = rospy.Subscriber('ar_tag_est', ArTagLocation, tag_callback)

    # Define target waypoint and trajectory publishers
    target_pose_pub = rospy.Publisher('position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size = 1, latch = True)
    target_traj_pub = rospy.Publisher('tracker/input_trajectory', MultiDOFJointTrajectory, queue_size = 1, latch = True)

    # Define ball drop start position publisher
    start_pose_pub = rospy.Publisher('launch/start_pose', PoseStamped, queue_size = 1, latch = True)

    # Define launch service proxies
    launch_start_serv = rospy.ServiceProxy('launch_start',LaunchStart)
    launch_start_input = LaunchStartRequest()
    launch_maneuver_serv = rospy.ServiceProxy('launch_maneuver',LaunchManeuver)
    launch_maneuver_input = LaunchManeuverRequest()

    # Define sweep service proxy
    search_routine_serv = rospy.ServiceProxy('search_routine',SearchRoutine)
    search_routine_input = SearchRoutineRequest()

    # Initialize publishing variables
    publish_target = False
    publish_traj = False
    last_msg = rospy.Time.now()
    ball_traj_gen = False
    search_traj_gen = False
    ball_traj_started = False
    search_traj_started = False

    state = rospy.wait_for_message('vehicle_state', String)

    while not rospy.is_shutdown():

        if state.data == 'takeoff':
            publish_target = False

        elif state.data == 'takeoff_complete':
            publish_target = False

        elif state.data == 'trans_12':
            now = rospy.Time.now()
            publish_traj = False
            if (now.secs - last_msg.secs > 0.5):
                publish_target = True
            setpoint_pose = planner_pose

        elif state.data == 'trans_23':
            now = rospy.Time.now()
            publish_traj = False
            if (now.secs - last_msg.secs > 0.5):
                publish_target = True
            setpoint_pose = planner_pose

        elif state.data == 'ar_search':
            now = rospy.Time.now()
            publish_traj = True
            if not search_traj_gen:
                search_traj_gen = True
                search_routine_input.x_bounds = [1.,12.5];
                search_routine_input.y_bounds = [-7.5,7.5];
                search_routine_input.z_bounds = [2.,4.];
                search_routine_input.wall_dist = 3.
                search_routine_input.vert_range = 1.
                search_routine_input.ccw_flag = False
                search_routine = search_routine_serv(search_routine_input)
            if not search_traj_started:
                publish_target = True
                search_traj_started = True
            else:
                publish_target = False
            setpoint_traj = search_routine.trajectory

        elif state.data == 'ar_refine':
            now = rospy.Time.now()
            publish_traj = False
            if (now.secs - last_msg.secs > 1.):
                publish_target = True
            setpoint_pose.pose.position.x = target_position.x + wall_normal.x*refine_spacing
            setpoint_pose.pose.position.y = target_position.y + wall_normal.y*refine_spacing
            setpoint_pose.pose.position.z = target_position.z + wall_normal.z*refine_spacing
            setpoint_pose.pose.orientation = quat_from_normal(wall_normal)

        elif state.data == 'trans_to_drop':
            now = rospy.Time.now()
            publish_traj = False
            if not ball_traj_gen:
                ball_traj_gen = True
                launch_start_input.target_position = target_position
                launch_start_input.wall_normal = wall_normal
                launch_start = launch_start_serv(launch_start_input)
                start_pose_pub.publish(launch_start.start_point)
                publish_target = True
            if (now.secs - last_msg.secs > 1.):
                publish_target = True
            setpoint_pose = launch_start.start_point

        elif state.data == 'ball_drop':
            publish_traj = False
            publish_target = False
            launch_maneuver_input.target_position = target_position
            launch_maneuver_input.wall_normal = wall_normal
            launch_maneuver = launch_maneuver_serv(launch_maneuver_input)

        if publish_target:
            if publish_traj:
                target_traj_pub.publish(setpoint_traj)
            else:
                target_traj_pub.publish(MultiDOFJointTrajectory())
                setpoint_point.transforms = [Transform(translation=Vector3(setpoint_pose.pose.position.x,\
                setpoint_pose.pose.position.y,setpoint_pose.pose.position.z),\
                rotation=setpoint_pose.pose.orientation)]
                target_pose_pub.publish(setpoint_point)
            publish_target = False
            last_msg = rospy.Time.now()

        rate.sleep()

if __name__ == '__main__':
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
