#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from qforge_ros.msg import ArTagLocation
from tf.transformations import quaternion_matrix
from tf import transformations
# try:
#     from cv_bridge import CvBridge
#     from cv2 import imshow
#     import cv2 as cv
# except ImportError:
#     rospy.logerr("Open CV dependency not met. Please run 'pip install opencv-python' or rebuild the image")
from cv_bridge import CvBridge
from cv2 import imshow
import cv2 as cv

# Fetch node rate parameter
arTag_rate = rospy.get_param('arTag_rate',50)
Pmin = rospy.get_param('arTag_lock_lim',5e-1)
    

def arTag_detect():
    # Initialize node
    rospy.init_node('ar_tag_detect')
    rate = rospy.Rate(arTag_rate)
    bridge = CvBridge()
    # Define tag estimate publisher
    tag_pub = rospy.Publisher('/ar_point', Point, queue_size = 1)
    image_pub = rospy.Publisher('tag_image_annotated', Image, queue_size = 1)
    method = cv.TM_SQDIFF_NORMED
    template = cv.imread('/root/uav_ws/src/qforge_ros/images/alvar.png', cv.IMREAD_COLOR)
    width = int(template.shape[1] * 70 / 100)
    height = int(template.shape[0] * 70 / 100)
    dim = (width, height)
    template = cv.resize(template, dim, interpolation = cv.INTER_AREA)


    template_lst = []

    scale_step = 1 # percent
    scale_min  = 5.0 # percent

    for i in range(0, int((100-scale_min)/scale_step)):

        width = int(template.shape[1] * (100-scale_step*i) / 100)
        height = int(template.shape[0] * (100-scale_step*i) / 100)
        dim = (width, height)
        rsz = cv.resize(template, dim, interpolation = cv.INTER_AREA)
        lst = [[width, height], rsz]
        template_lst.append(lst)
        # print((100-scale_step*i))


    while not rospy.is_shutdown():
       
        
        snap = rospy.wait_for_message("red/camera/color/image_raw",Image)
        img = bridge.imgmsg_to_cv2(snap)
        # img = cv.cvtColor(bridge.imgmsg_to_cv2(snap), cv.COLOR_BGR2GRAY)

        meth = 'cv.TM_CCORR_NORMED'

        method = eval(meth)
        # Apply template Matching

        maxV = 0.0

        for i in range(0, len(template_lst)):
            lst_i = template_lst[i]
            res = cv.matchTemplate(img,lst_i[1],method)
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
            if maxV < max_val:
                maxV = max_val
                indxMin = i
        
        lst_min = template_lst[i]
        dim_min = lst_min[0]
        w = dim_min[0]
        h = dim_min[1]
        
        res = cv.matchTemplate(img,lst_min[1],method)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)

        
        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc

        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv.rectangle(img,top_left, bottom_right, 255, 2)

        # print("x: " + str(top_left[0]) + " y: " + str(top_left[1]))
        # Step 3: Draw the rectangle on large_image
        rosimg = bridge.cv2_to_imgmsg(img, "bgr8")
        # cv.imshow('output',large_image)
        image_pub.publish(rosimg)
        # tag_pub.publish(msg)
            
            
if __name__ == '__main__':
    try:
        arTag_detect()
    except rospy.ROSInterruptException:
        pass