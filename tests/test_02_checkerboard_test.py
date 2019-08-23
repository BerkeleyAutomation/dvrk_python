import sys
sys.path.append('..')
import numpy as np
from dvrkClothSim import dvrkClothSim
from os import path

def transform_CB2PSM(x, y):
    if x>1: x==1.0
    if x<-1: x==-1.0
    if y>1:  y==1.0
    if y<-1: y==-1.0

    for i in range(raw_board):
        for j in range(column_board):
            if x == data_square[raw_board-1, j, 0] and y == data_square[i, column_board-1, 1]:  # corner point (x=1,y=1)
                return data_square[raw_board-1,column_board-1,2:5]
            else:
                if x == data_square[raw_board-1, j, 0]:  # border line of x-axis
                    if data_square[i, j, 1] <= y and y < data_square[i, j + 1, 1]:
                        y1 = data_square[raw_board-1, j, 1]
                        y2 = data_square[raw_board-1, j+1, 1]
                        Q11 = data_square[raw_board-1, j, 2:5]
                        Q12 = data_square[raw_board-1, j+1, 2:5]
                        return (y2-y)/(y2-y1)*Q11 + (y-y1)/(y2-y1)*Q12
                elif y == data_square[i, column_board-1, 1]:  # border line of y-axis
                    if data_square[i, j, 0] <= x and x < data_square[i + 1, j, 0]:
                        x1 = data_square[i, column_board-1, 0]
                        x2 = data_square[i+1, column_board-1, 0]
                        Q11 = data_square[i, column_board-1, 2:5]
                        Q21 = data_square[i+1, column_board-1, 2:5]
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

def move_cloth(x,y,dx,dy):
    pickup_pos = transform_CB2PSM(x,y)
    release_pos_temp = transform_CB2PSM(x+dx,y+dy)
    release_pos = np.array([release_pos_temp[0], release_pos_temp[1]])
    print pickup_pos, release_pos
    # just checking if the ROS input is fine
    user_input = raw_input("Are you sure the values to input to the robot arm?(y or n)")
    if user_input == "y":
        p.move_pose_pickup(pickup_pos, release_pos, 0, 'rad')

if __name__ == "__main__":
    raw_board = 6
    column_board = 6

    if path.exists('mapping_table'):
        # import data from file
        data_default = np.loadtxt("mapping_table", delimiter=',')
    else:
        # if file does not exist, set default
        data_default = np.zeros((raw_board*column_board,5))

    cnt = 0
    for i in range(raw_board):
        for j in range(column_board):
            data_default[cnt,0] = -1+j*0.4
            data_default[cnt,1] = -1+i*0.4
            cnt += 1
    data = data_default

    data_square = np.zeros((raw_board+1,column_board+1,5))
    for i in range(raw_board):
        for j in range(column_board):
            data_square[i,j,:] = data[column_board*j+i,0:5]

    for i in range(raw_board):
        data_square[i,column_board,:] = data_square[i,column_board-1,:]
    for j in range(column_board):
        data_square[raw_board,j] = data_square[raw_board-1,j]

    p = dvrkClothSim()
    p.set_position_origin([0.003, 0.001, -0.06], 0, 'deg')

    x = 0.2
    y = 0.2
    dx = 0.7
    dy = 0.7
    move_cloth(x,y,dx,dy)