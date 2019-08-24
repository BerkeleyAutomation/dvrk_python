#!/usr/bin/python
# -*- coding: utf-8 -*-
import argparse, cv2, math, os, rospy, sys, threading, time
from pprint import pprint
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from cv_bridge import CvBridge, CvBridgeError
from os.path import join
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
    """Process depth images the same as in the ISRR 2019 paper.

    Only applies if we're using depth images.
    """
    w,h = d_img.shape
    n_img = np.zeros([w, h, 3])
    d_img = d_img.flatten()
    d_img[d_img>cutoff] = 0.0
    d_img = d_img.reshape([w,h])
    for i in range(3):
        n_img[:, :, i] = d_img
    return n_img


def depth_3ch_to_255(d_img):
    """Process depth images the same as in the ISRR 2019 paper.

    Only applies if we're using depth images.
    """
    d_img = 255.0/np.max(d_img)*d_img
    d_img = np.array(d_img, dtype=np.uint8)
    for i in range(3):
        d_img[:, :, i] = cv2.equalizeHist(d_img[:, :, i])
    return d_img    


def process_img_for_net(c_img):
    """Do any sort of processing of the image for the neural network.

    For example, we definitely need to crop, and we may want to do some
    filtering or blurring to smoothen the texture. Our network uses images of
    size (100,100) but as long as we process it and then make sure it has the
    same height and width it'll be fine -- the net class has a resize command as
    a backup.

    Processing should be done before the cropping, because doing filtering after
    cropping results in very blurry images (the filters cover a wider range).
    """
    # TODO add processing, blurring, etc.
    c_img = c_img[175:675, 50:550]
    return c_img


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

        # Images are 1200 x 1920.
        assert c_img.shape == (1200, 1920, 3), c_img.shape
        c_img = process_img_for_net(c_img)
        assert c_img.shape[0] == c_img.shape[1], c_img.shape

        head = "/home/davinci0/adi/dvrk_python/dvrk_img"
        tail = "c_img_{}.png".format(str(i).zfill(2))
        img_path = join(head,tail)
        cv2.imwrite(img_path, c_img)
        print('  just saved: {}'.format(img_path))
        i += 1
        time.sleep(2)
    
    rospy.spin()
