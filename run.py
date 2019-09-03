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
import numpy as np
np.set_printoptions(suppress=True)
from collections import defaultdict
from os.path import join
# Stuff from our code base.
import utils as U
import config as C
from dvrkClothSim import dvrkClothSim
import camera


def _process_images(c_img, d_img, debug=True):
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

    # We fill in NaNs with zeros.
    c_img[np.isnan(c_img)] = 0
    d_img[np.isnan(d_img)] = 0

    # We `inpaint` to fill in the zero pixels, done on the raw depth values
    # before we convert to images. Requries some AUTOLAB dependencies.
    if camera.IN_PAINT:
        from perception import (ColorImage, DepthImage)
        print('in-painting the depth image (this may take a few seconds) ...')
        d_img = DepthImage(d_img)
        d_img = d_img.inpaint() # inpaint, then get back right away
        d_img = d_img.data # get raw numpy data back

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
                                     cutoff_min=camera.CUTOFF_MIN,
                                     cutoff_max=camera.CUTOFF_MAX)
    d_img_crop = camera.depth_3ch_to_255(d_img_crop)

    # Try blurring depth, bilateral recommends 9 for offline applications
    # that need heavy blurring. The two sigmas were 75 by default.
    d_img_crop_blur = cv2.bilateralFilter(d_img_crop, 9, 100, 100)
    #d_img_crop_blur = cv2.medianBlur(d_img_crop_blur, 5)

    # Let's save externally but we can do quick debugging here.
    U.save_image_numbers('tmp', img=c_img_crop, indicator='c_img', debug=True)
    U.save_image_numbers('tmp', img=d_img_crop, indicator='d_img', debug=True)

    return c_img_crop, d_img_crop


def run(args, cam, p, max_ep_length=10):
    """Run one episode, record statistics, etc."""
    stats = defaultdict(list)

    for i in range(max_ep_length):
        # STEP 1: query the image from the camera class using `cam`.
        c_img = None
        d_img = None
        while c_img is None:
            c_img = cam.read_color_data()
        while d_img is None:
            d_img = cam.read_depth_data()
        print('time {}, we have c_img and d_img'.format(i))

        # STEP 2: process image and save as a 100x100 png, see `camera.py` for some
        # tests. Image must be saved in specified DVRK_IMG_PATH for the net to see.
        c_img, d_img = _process_images(c_img, d_img)
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

        # STEP 3: get the output from the neural network loading class (you did
        # run it in a separate terminal tab, right?) and then show it to a human.
        # HUGE ASSUMPTION: that the last text file indicates the action we want.
        dvrk_action_paths = sorted(
                [join(C.DVRK_IMG_PATH,x) for x in os.listdir(C.DVRK_IMG_PATH) \
                    if x[-4:]=='.txt']
        )
        assert len(dvrk_action_paths) > 0, 'Did you run the neural net code?'
        action = np.loadtxt(dvrk_action_paths[-1])
        print('neural net says: {}'.format(action))

        # STEP 4. If the output would result in a dangerous position, human stops
        # by hitting ESC key. Otherwise, press any other key to continue.
        print(c_img.shape, d_img.shape)
        title = '{} -- ESC TO CANCEL'.format(action)
        if args.use_color:
            U.call_wait_key( cv2.imshow(title, c_img) )
        else:
            U.call_wait_key( cv2.imshow(title, d_img) )
        cv2.destroyAllWindows()

        # STEP 5: Watch the robot do its action. Terminate the script if the
        # resulting action makes things fail spectacularly.
        x  = action[0]
        y  = action[1]
        dx = action[2]
        dy = action[3]
        U.move_p_from_net_output(x, y, dx, dy,
                                 row_board=C.ROW_BOARD,
                                 col_board=C.COL_BOARD,
                                 data_square=C.DATA_SQUARE,
                                 p=p)

        # STEP 6. Record statistics. Sleep to let arm move out of the way.
        stats['actions'].append(action)
        time.sleep(4)

    # Final book-keeping and return statistics.
    print('All done with episode!')
    return stats


if __name__ == "__main__":
    # I would just set all to reasonable defaults, or put them in the config file.
    parser= argparse.ArgumentParser()
    parser.add_argument('--use_color', action='store_true')
    parser.add_argument('--trial', type=int, default=0)
    args = parser.parse_args()
    print('Running with arguments:\n{}'.format(args))

    # Setup
    p = dvrkClothSim()
    p.set_position_origin([0.003, 0.001, -0.060], 0, 'deg')
    cam = camera.RGBD()

    assert os.path.exists(C.CALIB_FILE), C.CALIB_FILE

    # Run one episode.
    stats = run(args, cam, p)

