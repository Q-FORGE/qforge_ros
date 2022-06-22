#!/usr/bin/env python

# Navigation node for qforge_ros package
# Publishes PoseStamped setpoint to 'tracker/input_pose'
# Publishes MultiDOFJointTrajectory trajectory setpoint to 'tracker/input_trajectory'
# Publishes PoseStamped ball drop start position to 'launch/start_pose'
# Publishes Bool initial_sweep_complete to 'nav/sweep_complete'
# Subscribes to String current vehicle state at 'vehicle_state'
# Subscribes to Odometry vehicle pose at 'mavros/global_position/local'
# Subscribes to ArTagLocation tag estimate at 'ar_tag_est'
# Subscribes to PoseStamped planner target pose at 'local_planner_manager/output_pose'
# Calls LaunchStart service at 'launch_start'
# Calls LaunchManeuver service at 'launch_maneuver'
# Calls SearchRoutine service at 'search_routine'
# Calls InitialSweep service at 'initial_sweep'

import rospy
from math import isclose
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion, Twist, Transform
from nav_msgs.msg import Odometry
from qforge_ros.srv import LaunchStart, LaunchStartRequest, \
        LaunchManeuver, LaunchManeuverRequest, \
        SearchRoutine, SearchRoutineRequest, \
        InitialSweep, InitialSweepRequest
from qforge_ros.msg import ArTagLocation
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

# Fetch node rate parameter
nav_rate = rospy.get_param('nav_rate',10)
refine_spacing = rospy.get_param('refine_spacing',3)

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
# planner_pose = PoseStamped()
planner_traj = MultiDOFJointTrajectory()

def pose_callback(msg):
    # Update current pose from mavros local position
    global current_pose
    current_pose.pose = msg.pose.pose
    current_pose.header = msg.header

def state_callback(msg):
    # Fetch most recent state from commander
    global state
    state = msg

def pathfinder_callback(msg):
    # Update local planner pose setpoint
    global planner_traj
    global new_astar_traj
    planner_traj = msg
    new_astar_traj = True

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
    global new_astar_traj

    # Initialize node
    rospy.init_node('nav')
    rate = rospy.Rate(nav_rate)

    # Define vehicle state and position subscribers
    state_sub = rospy.Subscriber('vehicle_state', String, state_callback)
    pose_sub = rospy.Subscriber('odometry', Odometry, pose_callback)
    planner_pose_sub = rospy.Subscriber('pathfinder/trajectory', MultiDOFJointTrajectory, pathfinder_callback)

    # Define ar tag location subscriber
    tag_sub = rospy.Subscriber('ar_tag_est', ArTagLocation, tag_callback)

    # Define target waypoint and trajectory publishers
    target_pose_pub = rospy.Publisher('tracker/input_pose', MultiDOFJointTrajectoryPoint, queue_size = 1, latch = True)
    target_traj_pub = rospy.Publisher('tracker/input_trajectory', MultiDOFJointTrajectory, queue_size = 1, latch = True)

    # Define ball drop start position publisher
    start_pose_pub = rospy.Publisher('launch/start_pose', PoseStamped, queue_size = 1, latch = True)

    # Define sweep complete publisher
    sweep_complete_pub = rospy.Publisher('nav/sweep_complete', Bool, queue_size = 1, latch = True)

    # Define launch service proxies
    launch_start_serv = rospy.ServiceProxy('launch_start',LaunchStart)
    launch_start_input = LaunchStartRequest()
    launch_maneuver_serv = rospy.ServiceProxy('launch_maneuver',LaunchManeuver)
    launch_maneuver_input = LaunchManeuverRequest()

    # Define sweep service proxies
    search_routine_serv = rospy.ServiceProxy('search_routine',SearchRoutine)
    search_routine_input = SearchRoutineRequest()
    initial_sweep_serv = rospy.ServiceProxy('initial_sweep',InitialSweep)
    initial_sweep_input = InitialSweepRequest()

    # Initialize publishing variables
    publish_target = False
    publish_traj = False
    last_msg = rospy.Time.now()
    ball_traj_gen = False
    search_traj_gen = False
    sweep_traj_gen = False
    ball_traj_started = False
    search_traj_started = False
    sweep_traj_started = False
    new_astar_traj = False

    state = rospy.wait_for_message('vehicle_state', String)

    while not rospy.is_shutdown():

        if state.data == 'takeoff':
            publish_target = False

        elif state.data == 'takeoff_complete':
            publish_target = False

        elif state.data == 'initial_sweep':
            now = rospy.Time.now()
            publish_traj = True
            if not sweep_traj_gen:
                sweep_traj_gen = True
                initial_sweep_input.y_bounds = [-7.5,7.5]
                initial_sweep_input.y_spacing = 4.25
                initial_sweep_input.initial_position.x = -10.
                initial_sweep_input.initial_position.y = 0.
                initial_sweep_input.initial_position.z = 3.
                initial_sweep = initial_sweep_serv(initial_sweep_input)
                initial_sweep_time = rospy.Time.now()
            if not sweep_traj_started:
                publish_target = True
                sweep_traj_started = True
            else:
                publish_target = False
            if (now.secs - initial_sweep_time.secs > 0.1):
                sweep_complete_pub.publish(True)
            setpoint_traj = initial_sweep.trajectory

        elif state.data == 'trans_12':
            now = rospy.Time.now()
            publish_traj = False
            if new_astar_traj:
                publish_target = True
                new_astar_traj = False
            setpoint_pose.pose.position.x = planner_traj.points[0].transforms[0].translation.x
            setpoint_pose.pose.position.y = planner_traj.points[0].transforms[0].translation.y
            setpoint_pose.pose.position.z = planner_traj.points[0].transforms[0].translation.z
            setpoint_pose.pose.orientation = planner_traj.points[0].transforms[0].rotation

        elif state.data == 'trans_23':
            now = rospy.Time.now()
            publish_traj = False
            if new_astar_traj:
                publish_target = True
                new_astar_traj = False
            setpoint_pose.pose.position.x = planner_traj.points[0].transforms[0].translation.x
            setpoint_pose.pose.position.y = planner_traj.points[0].transforms[0].translation.y
            setpoint_pose.pose.position.z = planner_traj.points[0].transforms[0].translation.z
            setpoint_pose.pose.orientation = planner_traj.points[0].transforms[0].rotation

        elif state.data == 'ar_search':
            now = rospy.Time.now()
            publish_traj = True
            if not search_traj_gen:
                search_traj_gen = True
                search_routine_input.x_bounds = [1.,12.5]
                search_routine_input.y_bounds = [-7.5,7.5]
                search_routine_input.z_bounds = [2.,3.5]
                search_routine_input.wall_dist = 4.
                search_routine_input.vert_range = 1.
                search_routine_input.ccw_flag = True
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
