import dvrk_get_pos
import dvrk_set_pos
import dvrk_get_joint
import dvrk_set_joint
import time
import rospy
import math

MILLION = 10**6
NODE_NAME = "test_node"

# Get pose (PSM1)
p_get = dvrk_get_pos.PositionSubscriber("PSM1",NODE_NAME,100)
p_get.rot_unit("deg")
p_get.start()

# Get pose (PSM2)
# p_get2 = dvrk_get_pos.PositionSubscriber("PSM2",NODE_NAME,100)
# p_get2.rot_unit("deg")
# p_get2.start()

# Get joint (PSM1)
# j_get = dvrk_get_joint.JointSubscriber("PSM1",NODE_NAME,100)
# j_get.rot_unit("deg")
# j_get.start()

# Get joint (PSM2)
# j_get2 = dvrk_get_joint.JointSubscriber("PSM2",NODE_NAME,100)
# j_get2.rot_unit("deg")
# j_get2.start()


# Set pose (PSM1)
# p_set = dvrk_set_pos.PositionPublisher("PSM1",NODE_NAME,10)
# pos_des = [0.0, 0.0, -0.13]  # Position (m)
# rot_des = [0, 0, 0]  # Euler angle ZYX (or roll-pitch-yaw)
# jaw_des = [0]
# p_set.start()
# p_set.move(pos_des, rot_des, jaw_des, "deg")

# Set pose (PSM2)
# p_set2 = dvrk_set_pos.PositionPublisher("PSM2",NODE_NAME,10)
# pos_des2 = [0.0, 0.0, -0.07]  # Position (m)
# rot_des2 = [0, 0, 0]  # Euler angle ZYX (or roll-pitch-yaw)
# jaw_des2 = [0]
# p_set2.start()
# p_set2.move(pos_des2, rot_des2, jaw_des2, "deg")

# Set joint (PSM1)
# j_set = dvrk_set_joint.JointPublisher("PSM1", NODE_NAME, 10)
# joint_des = [0, 0, 0.08, 0, 0, 0, 30]
# j_set.start()
# j_set.move(joint_des, "deg")

# Set joint (PSM2)
# j_set2 = dvrk_set_joint.JointPublisher("PSM2", NODE_NAME, 10)
# joint_des2 = [0, 0, 0.08, 0, 0, 0, 30]
# j_set2.start()
# j_set2.move(joint_des2, "deg")

time.sleep(2)
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
        # jaw_des = [amp_jaw*math.sin(2*math.pi*cnt/period_jaw)+20]    # (deg)
        # print(p_get.pos_curr, p_get.rot_curr, p_get.jaw_curr)
        # print(p_get2.pos_curr, p_get2.rot_curr, p_get2.jaw_curr)
        # print(j_get.joint_curr)
        # print(j_get2.joint_curr)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass