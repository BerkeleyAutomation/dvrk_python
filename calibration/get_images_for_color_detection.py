"""Do some quick testing, get images and then test color detection.

This is necessary for calibration later. Here, I just tune the color detection
script, or just save images and do the color detection elsewhere?

(c) November 2019 by Daniel Seita
"""
import os
import cv2
import pickle
import sys
import json
import argparse
import numpy as np
np.set_printoptions(suppress=True)
# Temporary hacks because we don't have this in a normal python package.
#sys.path.append('..')  # Allows: `python collect_calib_data.py
sys.path.append('.')   # Allows: `python calibration/collect_calib_data.py`

# Stuff from our code base.
import camera
import utils as U
from dvrkClothSim import dvrkClothSim


if __name__ == "__main__":
    # Set up the dvrk, close gripper, dvrk should be gripping some tape or ball.
    p = dvrkClothSim()
    p.arm.set_jaw(jaw=0)
    print('current pose:  {}'.format(p.arm.get_current_pose(unit='deg')))
    cam = camera.RGBD()
    print('Initialized `dvrkClothSim()`.\n')

    # Get image.
    c_img_raw = None
    d_img_raw = None
    while c_img_raw is None:
        c_img_raw = cam.read_color_data()
    while d_img_raw is None:
        d_img_raw = cam.read_depth_data()

    num = len([x for x in os.listdir('.') if 'c_img_' in x])
    cv2.imwrite('c_img_{}.png'.format(str(num).zfill(2)), c_img_raw)
