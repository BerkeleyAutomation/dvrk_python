#!/usr/bin/python
# -*- coding: utf-8 -*-
import argparse, cv2, math, os, rospy, sys, threading, time
from pprint import pprint
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from cv_bridge import CvBridge, CvBridgeError

#import tf
#import tf2_ros
#import tf2_geometry_msgs
#import IPython


class RGBD(object):
    def __init__(self):
        rospy.init_node("camera")
        # rostopic list [-s for subscribers] [-p for publishers] [-v verbose]
        self.bridge = CvBridge()
        self.img_rgb_raw = None
        self.img_depth_raw = None
        self.info = None
        self.is_updated = False

        # rospy.Subscriber(name, data_msg_class, callback)
        # Use `rqt_image_view` to see a interactive GUI of the possible rostopics

        #Adi: subscribing to the zivid depth topics (Da Vinci)
        self.sub_rgb_raw = rospy.Subscriber("zivid_camera/color/image_color", Image, self.callback_rgb_raw)
        self.sub_depth_raw = rospy.Subscriber("zivid_camera/depth/image_raw", Image, self.callback_depth_raw)
        self._sub_info = rospy.Subscriber("zivid_camera/color/camera_info", CameraInfo, self.callback_cam_info)

        #Code for the Fetch:
        #self.sub_rgb_raw = rospy.Subscriber('head_camera/rgb/image_raw', Image, self.callback_rgb_raw)
        #self.sub_depth_raw = rospy.Subscriber("head_camera/depth/image_raw", Image, self.callback_depth_raw)
        #self._sub_info = rospy.Subscriber("head_camera/rgb/camera_info", CameraInfo, self.callback_cam_info)

    def callback_rgb_raw(self, data):
        try:
            self.img_rgb_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.color_time_stamped = data.header.stamp
            self.is_updated = True
        except CvBridgeError as e:
            rospy.logerr(e)
            print(e)

    def callback_depth_raw(self, data):
        try:
            # you do not need to anything more than this to get the depth imgmsg as a cv2
            # make sure that you are using the right encoding
            self.img_depth_raw = self.bridge.imgmsg_to_cv2(data, '32FC1')
        except CvBridgeError as e:
            rospy.logerr(e)

    def callback_cam_info(self, data):
        try:
            self._info = data
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def read_color_data(self):
        return self.img_rgb_raw

    def read_depth_data(self):
        return self.img_depth_raw
        # return self.img_rgb_raw

    def read_info_data(self):
        return self._info

def depth_to_3ch(d_img, cutoff):
    w,h = d_img.shape
    n_img = np.zeros([w, h, 3])
    d_img = d_img.flatten()
    d_img[d_img>cutoff] = 0.0
    d_img = d_img.reshape([w,h])
    for i in range(3):
        n_img[:, :, i] = d_img
    return n_img

def depth_3ch_to_255(d_img):
    d_img = 255.0/np.max(d_img)*d_img
    d_img = np.array(d_img, dtype=np.uint8)
    for i in range(3):
        d_img[:, :, i] = cv2.equalizeHist(d_img[:, :, i])
    return d_img    



#d_img = None
#while d_img is None:
#print(d_img.shape)
#cv2.imwrite('d_img.png', d_img)
#np.save('d_img', d_img)

if __name__=='__main__':
    rgbd = RGBD()
    i = 0
    while i < 11:
        d_img = None
        c_img = None
    
        while d_img is None:
            d_img = rgbd.read_depth_data()
    
        while c_img is None:
            c_img = rgbd.read_color_data()

    
        d_img[np.isnan(d_img)] = 0
        c_img[np.isnan(c_img)] = 0
    
        d_img = depth_to_3ch(d_img, 1400)
        d_img = depth_3ch_to_255(d_img)

        #Images are 1920x1200
        #Crop the images here
        c_img = c_img[175:675, 50:550]

        #cv2.imwrite("d_img_0.png", d_img)
        cv2.imwrite("/home/davinci0/adi/dvrk_python/dvrk_img/c_img_" + str(i) + ".png", c_img)
        i += 1
        time.sleep(25)
    
    rospy.spin()
