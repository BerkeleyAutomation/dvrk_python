"""Adjust configuration here for settings.

This is used in the neural network loading code. THIS WILL REQUIRE PYTHON 3, so
DO NOT MERGE WITH OUR OTHER CONFIG FILE in the top-level repository.
"""
import os
import sys
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
DVRK_IMG_PATH = 'dir_for_imgs/'

# ---------------------------------------------------------------------------- #
# ADJUST WHICH NEURAL NETWORK WE WANT TO USE
# ---------------------------------------------------------------------------- #
HEAD = '/home/davinci0/seita/dvrk_python/nets/'

# Note: this net was not trained on heavier domain randomization ...
POLICY = 'openai-2019-08-30-21-39-05-868283/checkpoints/00199'

NET_FILE = join(HEAD, POLICY)


# ---------------------------------------------------------------------------- #
# ADJUST WHICH PATH WE WANT TO USE FOR TESTING
# ---------------------------------------------------------------------------- #
# This is only if we're testing some known images. Comment out if not.
#IMG_HEAD = '/home/davinci0/images'
#image_files = sorted(
#    [join(IMG_HEAD,x) for x in os.listdir(IMG_HEAD) \
#        if 'resized' in x and '.png' in x]
#)
#TEST_IMAGE_FILES = image_files[:20]
