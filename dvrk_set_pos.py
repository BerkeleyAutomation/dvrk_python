import rospy
import geometry_msgs.msg
import PyKDL
import numpy as np
from tf_conversions import posemath

def rad_to_deg(rad):
	return np.array(rad)*180./np.pi

def deg_to_rad(deg):
    return np.array(deg)*np.pi/180.

class PositionPublisher:
    def __init__(self):

        #========PUBLISHERS========#
        # position publishers
        self.pub = rospy.Publisher("/dvrk/PSM1/set_position_cartesian", geometry_msgs.msg.Pose, queue_size=1)
        rospy.init_node('position_publisher', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz

    def move(self,pos,rot):
        """
        Moves the PSM to the pose specified by (pos, rot).
        rot vector specifies RPY euler angles
        """
        px, py, pz = pos
        r, p, y = rot
        pos_ = PyKDL.Vector(px, py, pz)
        rot_ = PyKDL.Rotation.RPY(r, p, y)
        frame_des = PyKDL.Frame(rot_, pos_)
        temp = posemath.toMsg(frame_des)
        self.pub.publish(temp)
        #self.rate.sleep()

if __name__ == "__main__":
    p = PositionPublisher()
    pos = [0,0,0]
    rot = [0,0,0]  # roll, pitch, yaw
    while True:
        p.move(pos,rot)