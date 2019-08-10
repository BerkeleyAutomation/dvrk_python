import dvrkArm
import time
import math
import rospy

MILLION = 10**6

ps = dvrkArm.dvrkArm('/PSM1')
pos_des = [0.0, 0.0, -0.13]  # Position (m)
rot_des = [0, 0, 0]  # Euler angle ZYX (or roll-pitch-yaw)
jaw_des = [0]
ps.set_pose(pos_des, rot_des, 'deg')

cnt = 0.0
interval_ms = 10
amp_y = 0.02
amp_jaw = 20
period_y = 2.0
period_jaw = 1.0
rate = rospy.Rate(1000.0 / interval_ms)
t_sleep = 0.5
while not rospy.is_shutdown():
    try:
        cnt += 1000.0 / MILLION * interval_ms
        # pos_des = [0, amp_y*math.sin(2*math.pi*cnt/period_y), -0.07]  # (mm)
        # ps.set_pose(pos_des, rot_des, 'deg', False)
        print cnt

        rate.sleep()

        # action 1
        pos_des1 = [0, 0.03, -0.13]
        ps.set_pose(pos_des1, rot_des, "deg")
        jaw_des1 = [0]
        ps.set_jaw(jaw_des1, 'deg')

        pos_des2 = [0.03, 0.06, -0.13]
        ps.set_pose(pos_des2, rot_des, "deg")
        jaw_des2 = [40]
        ps.set_jaw(jaw_des2, 'deg')

        pos_des3 = [0.03, 0.06, -0.16]
        ps.set_pose(pos_des3, rot_des, "deg")
        jaw_des3 = [40]
        ps.set_jaw(jaw_des3, 'deg')

        pos_des4 = [0.03, 0.06, -0.16]
        ps.set_pose(pos_des4, rot_des, "deg")
        jaw_des4 = [0]
        ps.set_jaw(jaw_des4, 'deg')

        pos_des5 = [0.03, 0.06, -0.13]
        ps.set_pose(pos_des5, rot_des, "deg")
        jaw_des5 = [0]
        ps.set_jaw(jaw_des5, 'deg')
    except rospy.ROSInterruptException:
        pass