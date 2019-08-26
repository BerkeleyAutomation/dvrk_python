"""Test if calibration is working. This can be run as a stand alone file, i.e.:

python test_03_checkerboard_pick_test.py

should be enough. Be careful that the position is safe!
"""
import sys
sys.path.append('..')
import numpy as np
from dvrkClothSim import dvrkClothSim
from os import path
np.set_printoptions(suppress=True)
import utils as U
import time

if __name__ == "__main__":
    row_board = 6
    column_board = 6
    cloth_height = 0.005    # unit = (m)

    if path.exists('mapping_table'):
        # import data from file
        data_default = np.loadtxt("mapping_table", delimiter=',')
    else:
        # if file does not exist, set default
        data_default = np.zeros((row_board*column_board,5))

    cnt = 0
    for i in range(row_board):
        for j in range(column_board):
            data_default[cnt,0] = -1+j*0.4
            data_default[cnt,1] = -1+i*0.4
            data_default[cnt,4] = data_default[cnt,4] + cloth_height
            cnt += 1
    data = data_default

    data_square = np.zeros((row_board+1,column_board+1,5))
    for i in range(row_board):
        for j in range(column_board):
            data_square[i,j,:] = data[column_board*j+i,0:5]

    for i in range(row_board):
        data_square[i,column_board,:] = data_square[i,column_board-1,:]
    for j in range(column_board):
        data_square[row_board,j] = data_square[row_board-1,j]

    p = dvrkClothSim()
    p.set_position_origin([0.003, 0.001, -0.06], 0, 'deg')
    for i in range(row_board):
        for j in range(column_board):
            x = -1+j*0.4
            y = -1+i*0.4
            dx = 0.0
            dy = 0.0
            U.move_p_from_net_output(x, y, dx, dy, row_board, column_board, data_square, p)
            cnt += 1
            time.sleep(1)
