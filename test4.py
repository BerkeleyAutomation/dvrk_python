import dvrkArm
import time
import math
import rospy

MILLION = 10**6

ps = dvrkArm.dvrkArm('/PSM1')
pos_des = [0.0, 0.0, -0.13]  # Position (m)
rot_des = [0, 0, 0]  # Euler angle ZYX (or roll-pitch-yaw)
jaw_des = [50]
ps.set_pose(pos_des, rot_des, 'deg')

cnt = 0.0
interval_ms = 10
amp = 0.04
period = 3.0
rate = rospy.Rate(1000.0 / interval_ms)
t_sleep = 0.5
while not rospy.is_shutdown():
    try:
        ps.set_jaw(-5, 'deg')
        ps.set_jaw(50, 'deg')
        rate.sleep()
    except rospy.ROSInterruptException:
        pass