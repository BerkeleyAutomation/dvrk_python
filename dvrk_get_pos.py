import rospy
import geometry_msgs.msg
import numpy as np
from tf_conversions import posemath

def rad_to_deg(rad):
	return np.array(rad)*180./np.pi

def deg_to_rad(deg):
    return np.array(deg)*np.pi/180.

class PositionSubscriber():
    def __init__(self):

        #========SUBSCRIBERS========#
        # position subscriber
        rospy.init_node('position_subscriber', anonymous=True)
        rospy.Subscriber("/dvrk/PSM1/position_cartesian_current", geometry_msgs.msg.PoseStamped, self.position_callback,
                         queue_size=1)
        rospy.spin()

    def position_callback(self, msg):
        if rospy.is_shutdown():
            return
        # convert the pose into a kdl frame
        frame_curr = posemath.fromMsg(msg.pose)
        self.pos_curr = frame_curr.p
        self.rot_curr = rad_to_deg(np.array(frame_curr.M.GetEulerZYX()))
        print(self.pos_curr, self.rot_curr)

if __name__ == "__main__":
    ps = PositionSubscriber()