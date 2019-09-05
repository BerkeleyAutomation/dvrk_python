"""Use this for the main experiments. It runs one episode only.

NOTE: I am running into this error with a line in the camera file:

Traceback (most recent call last):
  File "run.py", line 73, in <module>
    cam = camera.RGBD()
  File "/home/davinci0/seita/dvrk_python/camera.py", line 18, in __init__
    rospy.init_node("camera")
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/client.py", line 274, in init_node
    raise rospy.exceptions.ROSException("rospy.init_node() has already been called with different arguments: "+str(_init_node_args))
rospy.exceptions.ROSException: rospy.init_node() has already been called with different arguments: ('dvrkArm_node', ['run.py'], True, 4, False, False

This code (`run.py`) will run fine if we just comment that line out, but I worry
if that functionality is needed.
"""
import argparse
import os
import sys
import time
import datetime
import cv2
import pickle
import numpy as np
np.set_printoptions(suppress=True)
from collections import defaultdict
from os.path import join
# Stuff from our code base.
import utils as U
import config as C
from dvrkClothSim import dvrkClothSim
import camera


def _process_images(c_img, d_img, args, debug=True):
    """Process images to make it suitable for deep neural networks.
    
    Mostly mirrors my tests in `camera.py`.
    """
    assert d_img.shape == (1200, 1920), d_img.shape
    assert c_img.shape == (1200, 1920, 3), c_img.shape

    if debug:
        nb_items = np.prod(np.shape(c_img))
        nb_not_nan = np.count_nonzero(~np.isnan(c_img))
        perc = nb_not_nan/float(nb_items)*100
        print('RGB image shape {}, has {} items'.format(c_img.shape, nb_items))
        print('  num NOT nan: {}, or {:.2f}%'.format(nb_not_nan, perc))
        nb_items = np.prod(np.shape(d_img))
        nb_not_nan = np.count_nonzero(~np.isnan(d_img))
        perc = nb_not_nan/float(nb_items)*100
        print('depth image shape {}, has {} items'.format(d_img.shape, nb_items))
        print('  num NOT nan: {}, or {:.2f}%'.format(nb_not_nan, perc))
    c_img[np.isnan(c_img)] = 0
    d_img[np.isnan(d_img)] = 0

    # We `inpaint` to fill in the zero pixels, done on raw depth values. Skip if
    # we're not doing color images due to time? Though we have to be careful if
    # we want to report both color/depth together?
    if C.IN_PAINT and (not args.use_color):
        d_img = U.inpaint_depth_image(d_img)

    # Process image, but this really means cropping!
    c_img_crop = camera.process_img_for_net(c_img)
    d_img_crop = camera.process_img_for_net(d_img)
    assert c_img_crop.shape[0] == c_img_crop.shape[1], c_img.shape

    # Check depth values (in meters), only for the CROPPED regions!
    if debug:
        print('\nAfter NaN filtering, ... for CROPPED depth image:')
        print('  max: {:.3f}'.format(np.max(d_img_crop)))
        print('  min: {:.3f}'.format(np.min(d_img_crop)))
        print('  mean: {:.3f}'.format(np.mean(d_img_crop)))
        print('  medi: {:.3f}'.format(np.median(d_img_crop)))
        print('  std: {:.3f}'.format(np.std(d_img_crop)))
        print('')

    # Let's process depth, from the cropped one, b/c we don't want values
    # out the cropped region to influence any depth 'scaling' calculations.
    d_img_crop = camera.depth_to_3ch(d_img_crop,
                                     cutoff_min=C.CUTOFF_MIN,
                                     cutoff_max=C.CUTOFF_MAX)
    d_img_crop = camera.depth_3ch_to_255(d_img_crop)

    # Try blurring depth, bilateral recommends 9 for offline applications
    # that need heavy blurring. The two sigmas were 75 by default.
    d_img_crop_blur = cv2.bilateralFilter(d_img_crop, 9, 100, 100)
    #d_img_crop_blur = cv2.medianBlur(d_img_crop_blur, 5)

    # Let's save externally but we can do quick debugging here.
    U.save_image_numbers('tmp', img=c_img_crop, indicator='c_img', debug=True)
    U.save_image_numbers('tmp', img=d_img_crop, indicator='d_img', debug=True)

    return c_img_crop, d_img_crop


def run(args, cam, p):
    """Run one episode, record statistics, etc."""
    stats = defaultdict(list)

    for i in range(args.max_ep_length):
        print('\n*************************************')
        print('ON TIME STEP (I.E., ACTION) NUMBER {}'.format(i+1))
        print('*************************************\n')

        # ----------------------------------------------------------------------
        # STEP 1: query the image from the camera class using `cam`. To avoid
        # the flashing strobe light, you have to move to the tab with the camera.
        # ----------------------------------------------------------------------
        c_img = None
        d_img = None
        print('Waiting for c_img, & d_img; please press ENTER in the appropriate tab')
        while c_img is None:
            c_img = cam.read_color_data()
        while d_img is None:
            d_img = cam.read_depth_data()
        print('  obtained the c_img and d_img')

        # ----------------------------------------------------------------------
        # STEP 2: process image and save as a 100x100 png, see `camera.py` for some
        # tests. Image must be saved in specified DVRK_IMG_PATH for the net to see.
        # ----------------------------------------------------------------------
        c_img, d_img = _process_images(c_img, d_img, args)
        assert c_img.shape == (100,100,3), c_img.shape
        assert d_img.shape == (100,100,3), d_img.shape
        if args.use_color:
            c_tail = "c_img_{}.png".format(str(i).zfill(2))
            img_path = join(C.DVRK_IMG_PATH, c_tail)
            cv2.imwrite(img_path, c_img)
        else:
            d_tail = "d_img_{}.png".format(str(i).zfill(2))
            img_path = join(C.DVRK_IMG_PATH, d_tail)
            cv2.imwrite(img_path, d_img)
        print('  just saved to: {}'.format(img_path))
        print('  now wait a few seconds for network to run')
        time.sleep(5)

        # ----------------------------------------------------------------------
        # STEP 3: get the output from the neural network loading class (you did
        # run it in a separate terminal tab, right?) and then show it to a human.
        # HUGE ASSUMPTION: that the last text file indicates the action we want.
        # ----------------------------------------------------------------------
        dvrk_action_paths = sorted(
                [join(C.DVRK_IMG_PATH,x) for x in os.listdir(C.DVRK_IMG_PATH) \
                    if x[-4:]=='.txt']
        )
        assert len(dvrk_action_paths) > 0, 'Did you run the neural net code?'
        action = np.loadtxt(dvrk_action_paths[-1])
        print('neural net says: {}'.format(action))

        # ----------------------------------------------------------------------
        # STEP 4. If the output would result in a dangerous position, human
        # stops by hitting ESC key. Otherwise, press any other key to continue.
        # ALSO this is where the human should terminate the episode!
        # ----------------------------------------------------------------------
        print(c_img.shape, d_img.shape)
        title = '{} -- ESC TO CANCEL (Or if episode done)'.format(action)
        if args.use_color:
            exit = U.call_wait_key( cv2.imshow(title, c_img) )
        else:
            exit = U.call_wait_key( cv2.imshow(title, d_img) )
        cv2.destroyAllWindows()
        if exit:
            break

        # ----------------------------------------------------------------------
        # STEP 5: Watch the robot do its action. Terminate the script if the
        # resulting action makes things fail spectacularly.
        # ----------------------------------------------------------------------
        x  = action[0]
        y  = action[1]
        dx = action[2]
        dy = action[3]
        U.move_p_from_net_output(x, y, dx, dy,
                                 row_board=C.ROW_BOARD,
                                 col_board=C.COL_BOARD,
                                 data_square=C.DATA_SQUARE,
                                 p=p)

        # ----------------------------------------------------------------------
        # STEP 6. Record statistics. Sleep just in case, also reset images.
        # ----------------------------------------------------------------------
        stats['actions'].append(action)
        stats['c_img'].append(c_img)
        stats['d_img'].append(d_img)
        cam.set_color_none()
        cam.set_depth_none()
        print('Reset color/depth in camera class, waiting a few seconds ...')
        time.sleep(3)

    # Final book-keeping and return statistics.
    if args.use_color:
        save_path = join('results', 'tier{}_color'.format(args.tier))
    else:
        save_path = join('results', 'tier{}_depth'.format(args.tier))
    if not os.path.exists(save_path):
        os.makedirs(save_path, exist_ok=True)
    count = len([x for x in os.listdir('results') if 'ep_' in x and '.pkl' in x])
    save_path = join(save_path, 'ep_{}.pkl'.format(str(count).zfill(3)))
    print('All done with episode! Saving stats to: {}'.format(save_path))
    with open(save_path, 'wb') as fh:
        pickle.dump(stats, fh)
    return stats


if __name__ == "__main__":
    # I would just set all to reasonable defaults, or put them in the config file.
    parser= argparse.ArgumentParser()
    #parser.add_argument('--use_color', action='store_true') # I'll forget ...
    parser.add_argument('--use_color', type=int) # 1 = True
    parser.add_argument('--tier', type=int)
    parser.add_argument('--max_ep_length', type=int, default=6)
    args = parser.parse_args()
    assert args.tier is not None
    assert args.use_color is not None
    print('Running with arguments:\n{}'.format(args))

    # Setup
    p = dvrkClothSim()
    p.set_position_origin([0.003, 0.001, -0.060], 0, 'deg')
    cam = camera.RGBD()

    assert os.path.exists(C.CALIB_FILE), C.CALIB_FILE

    # Run one episode.
    stats = run(args, cam, p)

