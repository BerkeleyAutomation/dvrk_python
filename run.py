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

# Stuff from our code base.
import utils as U
import config as C
from dvrkClothSim import dvrkClothSim
import camera


def run(args, cam, p):
    """Run one episode, record statistics, etc.
    """
    stats = defaultdict(list)

    # TODO Step 1: query the image from the camera class using `cam`.

    # TODO Step 2: process it and save as a 100x100 png image using
    # `camera.process_img_for_net()` ideally.

    # TODO Step 3: get the output from the neural network loading class (you did
    # run it in a separate terminal tab, right?) and then show it to the human.

    action = None #TODO

    # TODO Step 4. If the output would result in a dangerous position, human
    # terminates by hitting ESC key. Otherwise, press any other key to continue.

    # TODO Step 5: Watch the robot do its action. Terminate the script if the
    # resulting action makes things fail spectacularly.

    # Do something like:
    #x  = action[0]
    #y  = action[1]
    #dx = action[2]
    #dy = action[3]
    #U.move_p_from_net_output(x, y, dx, dy,
    #                         row_board=C.ROW_BOARD,
    #                         col_board=C.COL_BOARD,
    #                         data_square=data_square,
    #                         p=p)

    # TODO Step 6. Record statistics and book-keeping.
    stats['actions'].append(action)

    # TODO: repeat the steps above, unless we decide the episode should terminate.

    # Final book-keeping and return statistics.
    print('All done with episode!')
    return stats


if __name__ == "__main__":
    # I would just set all to reasonable defaults, or put them in the config file.
    pp = argparse.ArgumentParser()
    # Here's an example of how to add an argument:
    #pp.add_argument('--version', type=int, default=0)
    args = pp.parse_args()

    # Setup
    p = dvrkClothSim()
    p.set_position_origin([0.003, 0.001, -0.060], 0, 'deg')

    # TODO: figure out error, I'm getting:
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
