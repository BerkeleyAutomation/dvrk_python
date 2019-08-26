"""Test if calibration is working. This can be run as a stand alone file, i.e.:

python test_03_checkerboard_pick_test.py

should be enough. Be careful that the position is safe!
"""
import sys
sys.path.append('..')
import numpy as np
from dvrkClothSim import dvrkClothSim
np.set_printoptions(suppress=True)
import utils as U
import time

if __name__ == "__main__":

    row_board = 6
    column_board = 6
    cloth_height = 0.005    # unit = (m)
    data_square = U.load_mapping_table(row_board, column_board, 'mapping_table', cloth_height)
    p = dvrkClothSim()
    p.set_position_origin([0.003, 0.001, -0.06], 0, 'deg')
    for i in range(row_board):
        for j in range(column_board):
            x = -1+j*0.4
            y = -1+i*0.4
            dx = 0.0
            dy = 0.0
            U.move_p_from_net_output(x, y, dx, dy, row_board, column_board, data_square, p)
            time.sleep(1)
