#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes estimated artag location 'ar_tag_est'
# Subscribes to ar tag measurment 'ar_point'
# Subscribes to odometry '/red/odometry'

# from types import NoneType
import rospy
import numpy as np
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
pathfinder_rate = rospy.get_param('pathfinder_rate',0.5)

obs_sd = rospy.get_param('obs_safe_dist_m',1.1)
wall_sd_mod_frac = rospy.get_param('wall_safe_dist_mod_frac',0.5)

zone2_wi_x = 8
zone2_wi_y = 5

sparsity = 0.1

tlhc_world_x = 4
tlhc_world_y = 2.5

min_dist_pc2_sqr = 0.7**2 #m

bad_pc_tol = 0.05

global uav_pos, battleShip, zone_num
zone_num = 1
battleShip = BattleGrid(zone2_wi_x, zone2_wi_y, tlhc_world_x, tlhc_world_y, sparsity, True, obs_sd, wall_sd_mod_frac)

uav_pos = np.array([-1, 0, 0])


def pointcloud_callback(msg):
    # convert point cloud to grid points
    altitude_min = 1
    # xyz_array = ros_numpy.point_cloud2.get_xyz_points(msg.data)
    # xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):

            if (point[2] > altitude_min) and (point[3]<bad_pc_tol) and ((point[0]-uav_pos[0])**2 + (point[1]-uav_pos[1])**2 > min_dist_pc2_sqr):
                battleShip.add_world_to_grid(point[0],point[1])


def odometry_callback(msg):
    global uav_pos
    uav_pos = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,\
        msg.pose.pose.position.z])

def current_zone_callback(msg):
    global zone_num
    zone_num = msg.data


def pathfinder():
    # Initialize node
    rospy.init_node('pathfinder')
    rate = rospy.Rate(pathfinder_rate)

    traj_pub = rospy.Publisher('pathfinder/trajectory', MultiDOFJointTrajectory, queue_size = 1)
    path_pub = rospy.Publisher('pathfinder/trajectory_vis', Path, queue_size = 1)
    image_pub = rospy.Publisher('config_space_view', Image, queue_size = 1)

    bridge = CvBridge()

    rospy.Subscriber('/local_pointcloud', PointCloud2, pointcloud_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/hawk2/vrpn_client/estimated_odometry', Odometry, odometry_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('current_zone', Int16, current_zone_callback, queue_size=1, buff_size=2**24)


    alpha = 0.
    omega = 3.14/16.


    while not rospy.is_shutdown():
        if zone_num != 3:
            battleShip.getWaypoint(uav_pos[0],uav_pos[1],2,0)

            path = battleShip.refPath_world

            length = len(path)
            # print(length)
            ref_traj = MultiDOFJointTrajectory()
            vis_traj = Path()
            vis_traj.poses = []
            vis_traj.header.frame_id = "optitrack"
            ref_traj.points = []
            step_skip = 10
            look_ahead_factor = 1
            alpha = alpha + omega
            xi = 0.1*3.1415*np.sin(alpha)

            for i in range(15,min(step_skip*10,length-step_skip),step_skip):
                # ref_point = MultiDOFJointTrajectoryPoint()
                # ref_point.transforms = [Transform(translation=Vector3(path[i][0],path[i][1],3),rotation=Quaternion(0,0,0,1))]
                # # print(ref_point)

                ref_point = MultiDOFJointTrajectoryPoint()
                # xi = np.arctan2(path[i+step_skip][1]-path[i][1],path[i+step_skip][0]-path[i][0])
                # xi = 0
                ref_point.transforms = [Transform(translation=Vector3(path[i][0],path[i][1],1.5),rotation=Quaternion(0,0,-np.sin(xi/2),-np.cos(xi/2)))]
                # print(ref_point)

                ref_traj.points.append(ref_point)



                vis_point = PoseStamped()
                vis_point.pose.position.x = path[i][0]
                vis_point.pose.position.y = path[i][1]
                vis_point.pose.position.z = 1.5
                vis_point.pose.orientation.w = 1
                vis_point.pose.orientation.x = 0
                vis_point.pose.orientation.y = 0
                vis_point.pose.orientation.z = 0

                vis_traj.poses.append(vis_point)


            traj_pub.publish(ref_traj)
            image_pub.publish(bridge.cv2_to_imgmsg(cv.cvtColor(np.array(battleShip.config_space*255).astype('uint8'), cv.COLOR_GRAY2BGR), "bgr8"))
            path_pub.publish(vis_traj)
        # print(ref_traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        pathfinder()
    except rospy.ROSInterruptException:
        pass
