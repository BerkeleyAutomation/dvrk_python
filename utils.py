"""Shared methods, to be loaded in other code.
"""
import os
import sys
import cv2
import time
import numpy as np
from os import path
from os.path import join


# Useful constants.
ESC_KEYS = [27, 1048603]
MILLION = float(10**6)


def rad_to_deg(rad):
    return np.array(rad) * 180./np.pi


def deg_to_rad(deg):
    return np.array(deg) * np.pi/180.


def normalize(v):
    norm=np.linalg.norm(v, ord=2)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm


def save_image_numbers(head, img, indicator=None, debug=False):
    """Save image in a directory, but numbered at the end.
    
    Example, indicator might be `c_img`. Note: if we import os.path like `from
    os import path`, then please avoid name conflicts!
    """
    if indicator is None:
        nb = len([x for x in os.listdir(head) if '.png' in x])
        new_path = join(head, 'img_{}.png'.format(str(nb).zfill(4)))
    else:
        nb = len([x for x in os.listdir(head) if indicator in x])
        new_path = join(head, '{}_{}.png'.format(indicator, str(nb).zfill(4)))
    if debug:
        print('saving to: {}'.format(new_path))
    cv2.imwrite(new_path, img) 


def call_wait_key(nothing=None):
    """Call this like: `utils.call_wait_key( cv2.imshow(...) )`."""
    key = cv2.waitKey(0)
    if key in ESC_KEYS:
        print("Pressed ESC key. Terminating program...")
        sys.exit()


def inpaint_depth_image(d_img):
    """Inpaint depth image on raw depth values.

    Only import code here to avoid making them required if we're not inpainting.

    Also, inpainting is slow, so crop some irrelevant values. But BE CAREFUL!
    Make sure any cropping here will lead to logical consistency with the
    processing in `camera.process_img_for_net` later. For now we crop the 'later
    part' of each dimension, which still leads to > 2x speed-up.
    """
    d_img = d_img[:800,:1300]
    from perception import (ColorImage, DepthImage)
    print('now in-painting the depth image (shape {})...'.format(d_img.shape))
    start_t = time.time()
    d_img = DepthImage(d_img)
    d_img = d_img.inpaint()     # inpaint, then get d_img right away
    d_img = d_img.data          # get raw data back from the class
    cum_t = time.time() - start_t
    print('finished in-painting in {:.2f} seconds'.format(cum_t))
    return d_img


def load_mapping_table(row_board, column_board, file_name, cloth_height=0.005):
    """Load the mapping table which we need to map from neural net to action.

    The mapping table looks like this:

        nx,ny,rx,ry,rz

    Where `nx,ny` are coordinates w.r.t. the background plane, of which the
    cloth lies on top. Numbers range from (-1,1) and should be discretized in
    the mapping table. The `rx,ry,rz` are the x,y,z positions w.r.t. the robot's
    frame, and were derived by moving the robot gripper to that position over a
    checkerboard. Note that rotation is not included in the mapping table.

    :param row_board: number of rows.
    :param column_board: number of columns.
    :param file_name: name of the calibration file
    :param cloth_height: height offset, we add to the z values from the data.
    :return: data from calibration
    """
    if path.exists(file_name):
        # import data from file
        data_default = np.loadtxt(file_name, delimiter=',')
    else:
        # if file does not exist, set default
        data_default = np.zeros((row_board * column_board, 5))

    cnt = 0
    for i in range(row_board):
        for j in range(column_board):
            data_default[cnt, 0] = -1 + j * 0.4
            data_default[cnt, 1] = -1 + i * 0.4
            data_default[cnt, 4] = data_default[cnt, 4] + cloth_height
            cnt += 1
    data = data_default

    # Daniel: a bit confused about this, but it seems necessary to convert to
    # PSM space. See `transform_CB2PSM`.
    data_square = np.zeros((row_board + 1, column_board + 1, 5))
    for i in range(row_board):
        for j in range(column_board):
            data_square[i, j, :] = data[column_board * j + i, 0:5]

    for i in range(row_board):
        data_square[i, column_board, :] = data_square[i, column_board - 1, :]
    for j in range(column_board):
        data_square[row_board, j] = data_square[row_board - 1, j]

    return data_square


def transform_CB2PSM(x, y, row_board, col_board, data_square):
    """Minho's code, for calibation, figure out the PSM coordinates.

    Parameters (x,y) should be in [-1,1] (if not we clip it) and represent
    the coordinate range over the WHITE CLOTH BACKGROUND PLANE (or a
    'checkboard' plane). We then convert to a PSM coordinate.

    Uses bilinear interpolation.

    :param row_board: number of rows.
    :param col_board: number of columns.
    :param data_square: data from calibration.
    """
    if x>1: x=1.0
    if x<-1: x=-1.0
    if y>1:  y=1.0
    if y<-1: y=-1.0

    for i in range(row_board):
        for j in range(col_board):
            if x == data_square[row_board-1, j, 0] and y == data_square[i, col_board-1, 1]: # corner point (x=1,y=1)
                return data_square[row_board-1,col_board-1,2:5]
            else:
                if x == data_square[row_board-1, j, 0]:  # border line of x-axis
                    if data_square[i, j, 1] <= y and y < data_square[i, j + 1, 1]:
                        y1 = data_square[row_board-1, j, 1]
                        y2 = data_square[row_board-1, j+1, 1]
                        Q11 = data_square[row_board-1, j, 2:5]
                        Q12 = data_square[row_board-1, j+1, 2:5]
                        return (y2-y)/(y2-y1)*Q11 + (y-y1)/(y2-y1)*Q12
                elif y == data_square[i, col_board-1, 1]:  # border line of y-axis
                    if data_square[i, j, 0] <= x and x < data_square[i + 1, j, 0]:
                        x1 = data_square[i, col_board-1, 0]
                        x2 = data_square[i+1, col_board-1, 0]
                        Q11 = data_square[i, col_board-1, 2:5]
                        Q21 = data_square[i+1, col_board-1, 2:5]
                        return (x2-x)/(x2-x1)*Q11 + (x-x1)/(x2-x1)*Q21
                else:
                    if data_square[i,j,0] <= x and x < data_square[i+1,j,0]:
                        if data_square[i,j,1] <= y and y < data_square[i,j+1,1]:
                            x1 = data_square[i, j, 0]
                            x2 = data_square[i+1, j, 0]
                            y1 = data_square[i, j, 1]
                            y2 = data_square[i, j+1, 1]
                            Q11 = data_square[i, j, 2:5]
                            Q12 = data_square[i, j+1, 2:5]
                            Q21 = data_square[i+1, j, 2:5]
                            Q22 = data_square[i+1, j+1, 2:5]
                            if x1==x2 or y1==y2:
                                return []
                            else:
                                return 1/(x2-x1)/(y2-y1)*(Q11*(x2-x)*(y2-y) + Q21*(x-x1)*(y2-y) + Q12*(x2-x)*(y-y1) + Q22*(x-x1)*(y-y1))


def move_p_from_net_output(x, y, dx, dy, row_board, col_board, data_square, p,
                           debug=False):
    """Minho's code, for calibration, processes policy network output.

    Be careful, the x,y coordinate from the neural net refers to a coordinate
    range of [-1,1] in the x and y directions. Thus, (x,y) = (-1,-1) is the
    BOTTOM LEFT CORNER.

    However, in simulation, we first converted (x,y) into the range [0,1] by
    dividing by two (to get values in [-0.5,0.5]) and then adding 0.5. Then,
    given x and y values that ranged from [0,1], we deduced a dx and dy such
    that ... when we apply the action, dx and dy independently 'adjust' x and y.
    So it is indeed (x+dx) and (y+dy). To convert this to our case, it should be
    as simple as doubling the dx and dy values.
    
    It's a bit trick to understand by reading gym-cloth code, because I first
    convert dx and dy into other values, and then I repeatdly do motions until
    the full length is achieved.
    
    :params (x, y, dx, dy): outputs from the neural network, all in [-1,1].
    :param row_board: number of rows.
    :param col_board: number of columns.
    :param data_square: data from calibration, from `utils.load_mapping_table`.
    :param p: An instance of `dvrkClothSim`.
    """
    assert -1 <= x <= 1, x
    assert -1 <= y <= 1, y
    assert -1 <= dx <= 1, dx
    assert -1 <= dy <= 1, dy

    # Find the targets, and then get pose w.r.t. PSM.
    targ_x = x + 2*dx
    targ_y = y + 2*dy
    pickup_pos = transform_CB2PSM(x,
                                  y,
                                  row_board,
                                  col_board,
                                  data_square)
    release_pos_temp = transform_CB2PSM(targ_x,
                                        targ_y,
                                        row_board,
                                        col_board,
                                        data_square)

    release_pos = np.array([release_pos_temp[0], release_pos_temp[1]])
    if debug:
        print('pickup, release: {}, {}'.format(pickup_pos, release_pos))
    # just checking if the ROS input is fine
    # user_input = raw_input("Are you sure the values to input to the robot arm?(y or n)")
    # if user_input == "y":

    p.move_pose_pickup(pickup_pos, release_pos, 0, 'rad')

