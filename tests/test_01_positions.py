import sys
sys.path.append('.')
import dvrkArm
import time
import math
import rospy


if __name__ == "__main__":
    p = dvrkArm.dvrkArm('/PSM1')
    pose_frame = p.get_current_pose_frame()
    pose_rad = p.get_current_pose()
    pose_deg = p.get_current_pose(unit='deg')
    joint = p.get_current_joint(unit='deg')
    jaw = p.get_current_jaw(unit='deg')

    print('pose_frame:\n{}'.format(pose_frame))
    print('pose_rad: {}'.format(pose_rad))
    print('pose_deg: {}'.format(pose_deg))
    print('joint: {}'.format(joint))
    print('jaw: {}'.format(jaw))

    # Go to a pre-defined 'safe' pose.
    # pose_rad: (array([ 0.04845971,  0.05464517, -0.12231114]), array([ 4.65831872, -0.69974499,  0.87412989]))
    # pose_deg: (array([ 0.04845971,  0.05464517, -0.12231114]), array([ 266.90200204, -40.09243442,   50.08395329]))

    targ_pos = [0.04845971,  0.05464517, -0.12231114]
    targ_rot = [4.65831872, -0.69974499,  0.87412989]
    p.set_pose(pos=targ_pos, rot=targ_rot, unit='rad')

    # I.e., the gripper angle.
    p.set_jaw(jaw=10, unit='deg')

