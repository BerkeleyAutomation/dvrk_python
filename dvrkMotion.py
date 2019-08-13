import dvrkArm
import threading
import time
import math
import rospy
import numpy as np

from sensor_msgs.msg import JointState

MILLION = float(10**6)

class dvrkMotion(threading.Thread):
    """
    Motion library for dvrk
    """
    def __init__(self, interval_ms=10.0, ros_namespace='/dvrk'):
        threading.Thread.__init__(self)
        self.__ros_namespace = ros_namespace
        self.interval_ms = interval_ms
        self.arm = dvrkArm.dvrkArm('/PSM1')
        self.t = 0.0
        self.nStart = 0.0
        self.nEnd = 0.0
        self.stop_flag = False
        self.rate = rospy.Rate(1000.0 / self.interval_ms)

        # Motion variables
        self.pos_org = [0.0, 0.0, -0.13]  #   xyz position in (mm)
        self.rot_org = [0.0, 0.0, 0.0]    #   ZYX Euler angle in (rad)
        self.pos_pick = [0.0, 0.0]        #   xy coordinate for the cloth pick-up
        self.rot_pick = [0.0, 0.0, 0.0]   #   ZYX Euler angle in (rad)
        self.pick_depth = -0.16
        self.jaw_opening = 40*3.141592/180.0
        self.jaw_closing = 0*3.141592/180.0


    """
    Motion Creating function
    """
    def set_position_origin(self, pos, rot, unit='rad'):
        self.rot_org[0] = rot
        if unit == 'deg':
            self.rot_org[0] = rot*3.141592/180.0
        self.pos_org = pos

    def set_pose_pickup(self, pos_pick, pos_drop, rot_pick, unit='rad'):
        if unit == 'deg':
            rot_pick = rot_pick*3.141591/180.0

        # move to the origin
        self.arm.set_pose(self.pos_org, self.rot_org, 'rad')

        # move upon the pick-up spot and open the jaw
        p_temp = np.array([self.pos_org[0], self.pos_org[1], self.pos_org[2]]) + np.array([pos_pick[0], pos_pick[1], 0.0])
        r_temp = np.array(self.rot_org) + np.array([rot_pick, 0.0, 0.0])
        self.arm.set_pose(p_temp, r_temp, 'rad')
        self.arm.set_jaw(self.jaw_opening)

        # move downward and grasp the cloth
        pos_downward = [p_temp[0], p_temp[1], self.pick_depth]
        self.arm.set_pose(pos_downward, r_temp, 'rad')
        self.arm.set_jaw(self.jaw_closing)

        # move upward, move the cloth, and drop the cloth
        self.arm.set_pose(p_temp, r_temp, 'rad')
        p_temp2 = np.array([self.pos_org[0], self.pos_org[1], self.pos_org[2]]) + np.array([pos_drop[0], pos_drop[1], 0.0])
        self.arm.set_pose(p_temp2, r_temp, 'rad')
        self.arm.set_jaw(self.jaw_opening)

        # move to the origin and close the jaw
        self.arm.set_pose(self.pos_org, self.rot_org, 'rad')

    def start(self):
        self.stop_flag = False
        self.thread = threading.Thread(target=self.run, args=(lambda: self.stop_flag,))
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.stop_flag = True

    def run(self, stop):
        while True:
            self.nEnd = time.clock() * MILLION  # (us)
            if self.nEnd - self.nStart < self.interval_ms * 1000:
                pass
            else:
                # To do here
                self.t += 1000.0 / MILLION * self.interval_ms
                self.nStart = self.nEnd;
            if stop():
                break

    """
    Motion
    """
    # def sinusoidal_motion(self, amp=0.01, period=2.0, type='position_x'):
    #     if type == 'position_x':
    #         self.pos_des = amp*math.sin(2*math.pi*self.t/period)

    """
    Conversion function
    """
    def rad_to_deg(self,rad):
        return np.array(rad) *180./np.pi

    def deg_to_rad(self,deg):
        return np.array(deg) *np.pi/180.


if __name__ == "__main__":
    p = dvrkMotion()
    p.start()
    p.set_position_origin([-0.05, 0.02, -0.14], 0, 'deg')
    while True:
        p.set_pose_pickup([0.08, 0.0], [0.06, 0.02], 0, 'deg')
        p.set_pose_pickup([0.08, 0.07], [0.06, 0.05], 0, 'deg')
        p.set_pose_pickup([0.0, 0.07], [0.02, 0.05], 0, 'deg')
