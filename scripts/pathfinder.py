#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes estimated artag location 'ar_tag_est'
# Subscribes to ar tag measurment 'ar_point'
# Subscribes to odometry '/red/odometry'

# from types import NoneType
import rospy
import numpy as np
import ros_numpy
# import pyastar2d
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2
from BattleGrid import BattleGrid
from std_msgs.msg import String, Bool, Int16
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Vector3, Transform, Quaternion, PoseStamped
# from geometry_msgs.msg import PoseStamped 
from qforge_ros.msg import ArTagLocation
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tf.transformations import quaternion_matrix
from tf import transformations
try:
    from cv_bridge import CvBridge
    from cv2 import imshow
    import cv2 as cv
except ImportError:
    rospy.logerr("Open CV dependency not met. Please run 'pip install opencv-python' or rebuild the image")



# Fetch node rate parameter
pathfinder_rate = rospy.get_param('pathfinder_rate',5)

zone2_wi_x = 25
zone2_wi_y = 14

sparsity = 0.1

tlhc_world_x = 12.5
tlhc_world_y = 7.5

global uav_pos, battleShip
battleShip = BattleGrid(zone2_wi_x, zone2_wi_y, tlhc_world_x, tlhc_world_y, sparsity) 
uav_pos = np.array([-10, 0, 3])

def pointcloud_callback(msg):
    # convert point cloud to grid points
    altitude_min = 1
    # xyz_array = ros_numpy.point_cloud2.get_xyz_points(msg.data)
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            if point[2] > altitude_min:
                battleShip.add_world_to_grid(point[0],point[1])

    # sz = xyz_array.shape
    # rw = sz[0]
    # elements = sz[1] 
    # for i in range(0, elements):
    #     height = xyz_array[i,2]
    #     if height > altitude_min:
    #         rospy.logerr(xyz_array[i,0])
    #         rospy.logerr(xyz_array[i,1])
    #         rospy.logerr(xyz_array[i,2])
    #         rospy.logerr("-------------")
    #         battleShip.add_world_to_grid(xyz_array[i,0],xyz_array[i,1])

def odometry_callback(msg):
    global uav_pos
    uav_pos = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,\
        msg.pose.pose.position.z])
    

def pathfinder():
    # Initialize node
    rospy.init_node('pathfinder')
    rate = rospy.Rate(pathfinder_rate)

    traj_pub = rospy.Publisher('pathfinder/trajectory', MultiDOFJointTrajectory, queue_size = 1)
    path_pub = rospy.Publisher('pathfinder/trajectory_vis', Path, queue_size = 1)
    image_pub = rospy.Publisher('config_space_view', Image, queue_size = 1)

    bridge = CvBridge()

    rospy.Subscriber('/local_pointcloud', PointCloud2, pointcloud_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('odometry', Odometry, odometry_callback, queue_size=1, buff_size=2**24)


    while not rospy.is_shutdown():
        battleShip.getWaypoint(uav_pos[0],uav_pos[1],10,0)

        path = battleShip.refPath_world

        length = len(path)
        # print(length)
        ref_traj = MultiDOFJointTrajectory()
        vis_traj = Path()
        vis_traj.poses = []
        vis_traj.header.frame_id = "world"
        ref_traj.points = []
        for i in range(0,length):
            ref_point = MultiDOFJointTrajectoryPoint()
            ref_point.transforms = [Transform(translation=Vector3(path[i][0],path[i][1],3),rotation=Quaternion(0,0,0,1))]
            # print(ref_point)
            ref_traj.points.append(ref_point)

            vis_point = PoseStamped()
            vis_point.pose.position.x = path[i][0]
            vis_point.pose.position.y = path[i][1]
            vis_point.pose.position.z = 3
            vis_point.pose.orientation.w = 1
            vis_point.pose.orientation.x = 0
            vis_point.pose.orientation.y = 0
            vis_point.pose.orientation.z = 0

            vis_traj.poses.append(vis_point)

        
        traj_pub.publish(ref_traj)
        image_pub.publish(bridge.cv2_to_imgmsg(cv.cvtColor(np.array(battleShip.config_space_view*255).astype('uint8'), cv.COLOR_GRAY2BGR), "bgr8"))
        path_pub.publish(vis_traj)
        # print(ref_traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        pathfinder()
    except rospy.ROSInterruptException:
        pass
