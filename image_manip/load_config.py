"""Adjust configuration here for settings.
"""
import os
import sys
import cv2
import time
import pickle
import numpy as np
from os.path import join

BLUE = (255,0,0)
GREEN = (0,255,0)
RED = (0,0,255)

# ---------------------------------------------------------------------------- #
# WHERE THE DVRK CODE SAVES IMAGES
# ---------------------------------------------------------------------------- #
DVRK_IMG_PATH = 'dvrk_img/'

# ---------------------------------------------------------------------------- #
# ADJUST WHICH NEURAL NETWORK WE WANT TO USE
# ---------------------------------------------------------------------------- #
HEAD = '/home/seita/policies-cloth-sim/'

# Note: this net was not trained on heavier domain randomization ...
POLICY = 'openai-2019-08-17-17-30-14-472186/checkpoints/00400'

NET_FILE = join(HEAD, POLICY)


# ---------------------------------------------------------------------------- #
# ADJUST WHICH PATH WE WANT TO USE FOR TESTING
# ---------------------------------------------------------------------------- #
IMG_HEAD = '/home/seita/gym-cloth/logs'
image_files = sorted(
    [join(IMG_HEAD,x) for x in os.listdir(IMG_HEAD) \
        if 'resized' in x and '.png' in x]
)
TEST_IMAGE_FILES = image_files[:20]
