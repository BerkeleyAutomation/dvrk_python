"""Use this for the main experiments. It runs one episode only.
"""
import argparse
import os
import sys
import time
import datetime
import cv2
import numpy as np
np.set_printoptions(suppress=True)
from collections import defaultdict
from os.path import join
# Stuff from our code base.
import utils as U
import config as C
from dvrkClothSim import dvrkClothSim
import camera


def run(args, cam, p):
    """Run one episode, record statistics, etc.
    """
    stats = defaultdict(list)

    for i in range(10):
        # TODO Step 1: query the image from the camera class using `cam`.
        c_img = None
        while c_img is None:
            c_img = cam.read_color_data()
        c_img[np.isnan(c_img)] = 0

        # TODO Step 2: process it and save as a 100x100 png image using
        # `camera.process_img_for_net()` ideally.
        c_img = camera.process_img_for_net(c_img) #Crop the image and resize to 100x100
        head = C.DVRK_IMG_PATH
        tail = "c_img_{}.png".format(str(i).zfill(2))
        img_path = join(head,tail)
        cv2.imwrite(img_path, c_img)
        print('  just saved: {}'.format(img_path))
        time.sleep(5)

        # TODO Step 3: get the output from the neural network loading class (you did
        # run it in a separate terminal tab, right?) and then show it to the human.

        dvrk_action_paths = sorted([join(C.DVRK_IMG_PATH,x) for x in os.listdir(C.DVRK_IMG_PATH) if x[-4:]=='.txt'])
        action = np.loadtxt(dvrk_action_paths[-1]) #TODO

        # TODO Step 4. If the output would result in a dangerous position, human
        # terminates by hitting ESC key. Otherwise, press any other key to continue.
        #Adi: I say we just put a debugger here because we can't map action to pixels just yet
        #import ipdb; ipdb.set_trace() I'm unable to install ipdb on the system python for some reason

        # TODO Step 5: Watch the robot do its action. Terminate the script if the
        # resulting action makes things fail spectacularly.

        # Do something like:
        x  = action[0]
        y  = action[1]
        dx = action[2]
        dy = action[3]
        U.move_p_from_net_output(x, y, dx, dy,
                                 row_board=C.ROW_BOARD,
                                 col_board=C.COL_BOARD,
                                 data_square=data_square,
                                 p=p)

        # TODO Step 6. Record statistics and book-keeping.
        stats['actions'].append(action)

        # TODO: repeat the steps above, unless we decide the episode should terminate.

    # Final book-keeping and return statistics.
    print('All done with episode!')
    return stats


if __name__ == "__main__":
    # I would just set all to reasonable defaults, or put them in the config file.
    pp = argparse.ArgumentParser()
    #parser.add_argument('--use_color', action='store_true')
    #pp.add_argument('--version', type=int, default=0)
    args = pp.parse_args()

    # Setup
    p = dvrkClothSim()
    p.set_position_origin([0.003, 0.001, -0.060], 0, 'deg')

    # NOTE: figure out error, I'm getting:
    #
    # Traceback (most recent call last):
    #   File "run.py", line 73, in <module>
    #     cam = camera.RGBD()
    #   File "/home/davinci0/seita/dvrk_python/camera.py", line 18, in __init__
    #     rospy.init_node("camera")
    #   File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/client.py", line 274, in init_node
    #     raise rospy.exceptions.ROSException("rospy.init_node() has already been called with different arguments: "+str(_init_node_args))
    # rospy.exceptions.ROSException: rospy.init_node() has already been called with different arguments: ('dvrkArm_node', ['run.py'], True, 4, False, False
    #
    # This code will run fine if we just comment that line out, but I worry if
    # that functionality is needed.
    cam = camera.RGBD()

    assert os.path.exists(C.CALIB_FILE), C.CALIB_FILE
    calib = np.loadtxt(C.CALIB_FILE, delimiter=',')

    # Run one episode.
    stats = run(args, cam, p)

    # TODO: Save the stats, do some reporting, etc.
