import numpy as np
import rospy
from matplotlib import pyplot as plt
from sensor_msgs.msg import JointState

def plot(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(time, msg.position[6], 'bo')
        # plt.plot(time, msg.position[6], 'bo')

        # print(msg.position)
        # plt.axis([0, 10, -0.2, 0.2])
        plt.draw()
        plt.pause(0.00000000001)
    counter += 1

if __name__ == '__main__':
    counter = 0
    rospy.init_node("plotter")
    rospy.Subscriber("/dvrk/PSM1/io/actuator_current_measured", JointState, plot)
    plt.ion()
    plt.show()
    rospy.spin()