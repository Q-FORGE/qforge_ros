#!/usr/bin/env python
# demo to test search service and AR detection

import rospy
import std_msgs.msg
from geometry_msgs.msg import Transform,Quaternion,Twist,Vector3,PoseStamped,Pose,Point
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from qforge_ros.srv import SearchRoutine, SearchRoutineRequest
from qforge_ros.msg import ArTagLocation

# Fetch node rate parameter
demo_rate = rospy.get_param('demo_rate',10)

def ar_est_callback(msg):
    # Update current pose from mavros local position
    global ar_estimate
    ar_estimate.lock = msg.lock
    ar_estimate.position = msg.position
    ar_estimate.normal = msg.normal
    

def demo_search():
    # Define AR tag suscriber
    ar_est_sub = rospy.Subscriber('/red/ar_tag_est', ArTagLocation , ar_est_callback)

    rospy.wait_for_service('search_routine')
    search_routine_handle = rospy.ServiceProxy('search_routine',SearchRoutine)
    search_routine_input = SearchRoutineRequest()
    search_routine_input.min_bound = Vector3(2,-6.5,2)
    search_routine_input.max_bound = Vector3(12,6.5,4.5)
    search_routine_input.fov = Vector3(4,4,1)
    # search_routine_input.translational_time = 2.0
    # search_routine_input.rotational_time = 0.5    
    
    search_routine = search_routine_handle(search_routine_input)

    rospy.init_node('demo_search')
    rate = rospy.Rate(demo_rate)


    trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)
    trajectory_pub.publish(search_routine.trajectory)
    rospy.sleep(5)

    while not rospy.is_shutdown():
        ar_est_sub = rospy.Subscriber('ar_tag_est', ArTagLocation , ar_est_callback)
        ar_lock = ar_estimate.lock
        if ar_lock:
            # ar_position = [ar_estimate.position.x,ar_estimate.position.y,ar_estimate.position.z]
            # ar_normal = [ar_estimate.normal.x,ar_estimate.normal.y,ar_estimate.normal.z]

            final_pose = MultiDOFJointTrajectoryPoint()
            final_pose.transforms = [Transform(translation=Vector3(9,-3,2),rotation=Quaternion(0,0,0,1))]
            final_trajectory = MultiDOFJointTrajectory()
            final_trajectory.points = [final_pose]

            trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)
            trajectory_pub.publish(final_trajectory)
            rate.sleep()

if __name__ == '__main__':
    try:
        demo_search()
    except rospy.ROSInterruptException:
        pass



# message_test = SearchRoutineResponse

