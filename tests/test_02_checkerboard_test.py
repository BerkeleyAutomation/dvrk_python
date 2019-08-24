"""Test if calibration is working. This can be run as a stand alone file, i.e.:

python test_02_checkerboard_test.py

should be enough. Be careful that the position is safe!
"""
import sys
sys.path.append('..')
import numpy as np
from dvrkClothSim import dvrkClothSim
from os import path
np.set_printoptions(suppress=True)
import utils as U


if __name__ == "__main__":
    row_board = 6
    column_board = 6

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

    x = 0.2
    y = 0.2
    dx = 0.7
    dy = 0.7
    U.move_p_from_net_output(x, y, dx, dy,
                             row_board=row_board,
                             col_board=column_board,
                             data_square=data_square,
                             p=p)
