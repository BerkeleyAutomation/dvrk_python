import threading
import time
import rospy
import math
import dvrkArm

MILLION = 10**6

class dvrkMotion(threading.Thread):
    """
    Motion library for dvrk
    """
    def __init__(self, interval_ms=10.0):
        self.rate = rospy.Rate(1000.0 / interval_ms)
        self.interval_ms = interval_ms
        self.cnt = 0.0

        self.arm1 = dvrkArm('/PSM1')
        self.arm2 = dvrkArm('/PSM2')

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('dvrkMotion_node', anonymous=True, log_level=rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')
        threading.Thread.__init__(self)

    def start(self):
        pos_des = [0.0, 0.0, -0.13]  # Position (m)
        rot_des = [0, 0, 0]  # Euler angle ZYX (or roll-pitch-yaw)
        jaw_des = [0]
        self.p_set.start()
        self.p_set.move(pos_des, rot_des, jaw_des, "deg")

        self.stop_flag = False
        self.thread = threading.Thread(target=self.run, args=(lambda: self.stop_flag,))
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.stop_flag = True

    def run(self, stop):
        while not rospy.is_shutdown():
            # To do


            self.cnt += 1000.0 / MILLION * self.interval_ms
            self.rate.sleep()
            if stop():
                break

if __name__ == "__main__":
    p = dvrkClothMotion("cloth_exp_node", 10)




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
        # action 1
        pos_des1 = [0, 0.03, -0.13]
        jaw_des1 = [0]
        p_set.move(pos_des1, rot_des, jaw_des1, "deg")
        time.sleep(t_sleep)

        pos_des2 = [0.03, 0.06, -0.13]
        jaw_des2 = [40]
        p_set.move(pos_des2, rot_des, jaw_des2, "deg")
        time.sleep(t_sleep)

        pos_des3 = [0.03, 0.06, -0.16]
        jaw_des3 = [40]
        p_set.move(pos_des3, rot_des, jaw_des3, "deg")
        time.sleep(t_sleep)

        pos_des4 = [0.03, 0.06, -0.16]
        jaw_des4 = [0]
        p_set.move(pos_des4, rot_des, jaw_des4, "deg")
        time.sleep(t_sleep)

        pos_des5 = [0.03, 0.06, -0.13]
        jaw_des5 = [0]
        p_set.move(pos_des5, rot_des, jaw_des5, "deg")
        time.sleep(t_sleep)

        # action 2
        pos_des1 = [0, 0.03, -0.13]
        jaw_des1 = [0]
        p_set.move(pos_des1, rot_des, jaw_des1, "deg")
        time.sleep(t_sleep)

        pos_des2 = [-0.03, 0.06, -0.13]
        jaw_des2 = [40]
        p_set.move(pos_des2, rot_des, jaw_des2, "deg")
        time.sleep(t_sleep)

        pos_des3 = [-0.03, 0.06, -0.16]
        jaw_des3 = [40]
        p_set.move(pos_des3, rot_des, jaw_des3, "deg")
        time.sleep(t_sleep)

        pos_des4 = [-0.03, 0.06, -0.16]
        jaw_des4 = [0]
        p_set.move(pos_des4, rot_des, jaw_des4, "deg")
        time.sleep(t_sleep)

        pos_des5 = [-0.03, 0.06, -0.13]
        jaw_des5 = [0]
        p_set.move(pos_des5, rot_des, jaw_des5, "deg")
        time.sleep(t_sleep)

        # action 3
        pos_des6 = [0, 0.03, -0.13]
        jaw_des6 = [0]
        p_set.move(pos_des6, rot_des, jaw_des6, "deg")
        time.sleep(t_sleep)

        pos_des7 = [-0.03, 0, -0.13]
        jaw_des7 = [40]
        p_set.move(pos_des7, rot_des, jaw_des7, "deg")
        time.sleep(t_sleep)

        pos_des8 = [-0.03, 0, -0.16]
        jaw_des8 = [40]
        p_set.move(pos_des8, rot_des, jaw_des8, "deg")
        time.sleep(t_sleep)

        pos_des9 = [-0.03, 0, -0.16]
        jaw_des9 = [0]
        p_set.move(pos_des9, rot_des, jaw_des9, "deg")
        time.sleep(t_sleep)

        pos_des10 = [-0.03, 0, -0.13]
        jaw_des10 = [0]
        p_set.move(pos_des10, rot_des, jaw_des10, "deg")
        time.sleep(t_sleep)

        # action 4
        pos_des11 = [0, 0.03, -0.13]
        jaw_des11 = [0]
        p_set.move(pos_des11, rot_des, jaw_des11, "deg")
        time.sleep(t_sleep)

        pos_des12 = [0.03, 0, -0.13]
        jaw_des12 = [40]
        p_set.move(pos_des12, rot_des, jaw_des12, "deg")
        time.sleep(t_sleep)

        pos_des13 = [0.03, 0, -0.16]
        jaw_des13 = [40]
        p_set.move(pos_des13, rot_des, jaw_des13, "deg")
        time.sleep(t_sleep)

        pos_des14 = [0.03, 0, -0.16]
        jaw_des14 = [0]
        p_set.move(pos_des14, rot_des, jaw_des14, "deg")
        time.sleep(t_sleep)

        pos_des15 = [0.03, 0, -0.13]
        jaw_des15 = [0]
        p_set.move(pos_des15, rot_des, jaw_des15, "deg")
        time.sleep(t_sleep)

        # pos_des = [0, amp_y*math.sin(2*math.pi*cnt/period_y), -0.07]  # (mm)
        # jaw_des = [amp_jaw*math.sin(2*math.pi*cnt/period_jaw)+20]    # (deg)
        # print(p_get.pos_curr, p_get.rot_curr, p_get.jaw_curr)
        # print(p_get2.pos_curr, p_get2.rot_curr, p_get2.jaw_curr)
        # print(j_get.joint_curr)
        # print(j_get2.joint_curr)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass