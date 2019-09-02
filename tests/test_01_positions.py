"""
Use this script to simply get the pose of the dvrk. Move it to a spot, record
pose, etc. This is to confirm if real code will do the same thing.

python test_01_positions.py
"""
import sys
sys.path.append('..')
from dvrkArm import dvrkArm
import time
import math
import rospy


if __name__ == "__main__":
    p = dvrkArm('/PSM2')
    pose_frame = p.get_current_pose_frame()
    pose_rad = p.get_current_pose()
    pose_deg = p.get_current_pose(unit='deg')
    jaw = p.get_current_jaw(unit='deg')

    print('pose_frame:\n{}'.format(pose_frame))
    print('pose_rad: {}'.format(pose_rad))
    print('pose_deg: {}'.format(pose_deg))
    print('jaw: {}'.format(jaw))

    # was this deprecated?
    #joint = p.get_current_joint(unit='deg')
    #print('joint: {}'.format(joint))

    # Go to a pre-defined 'safe' pose.
    # pose_rad: (array([ 0.04845971,  0.05464517, -0.12231114]), array([ 4.65831872, -0.69974499,  0.87412989]))
    # pose_deg: (array([ 0.04845971,  0.05464517, -0.12231114]), array([ 266.90200204, -40.09243442,   50.08395329]))

    #targ_pos = [0.04845971,  0.05464517, -0.12231114]
    #targ_rot = [4.65831872, -0.69974499,  0.87412989]

    # I.e., the gripper angle.
    #p.set_jaw(jaw=10, unit='deg')

