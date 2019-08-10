import threading
import rospy

import numpy as np
import PyKDL

from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32, Empty, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

class dvrkArm(object):
    """Simple arm API wrapping around ROS messages
    """
    def __init__(self, arm_name, ros_namespace='/dvrk'):

        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace
        self.__goal_reached = False
        self.__goal_reached_event = threading.Event()

        # continuous publish from dvrk_bridge
        self.__position_cartesian_current = PyKDL.Frame()
        self.__position_joint_current = np.array(0, dtype = np.float)
        self.__position_jaw_current = 0.0

        self.__sub_list = []
        self.__pub_list = []

        # publisher
        frame = PyKDL.Frame()
        self.__full_ros_namespace = self.__ros_namespace + self.__arm_name
        self.__set_position_joint_pub = rospy.Publisher(self.__full_ros_namespace + '/set_position_joint', JointState,
                                                        latch = True, queue_size = 1)
        self.__set_position_goal_joint_pub = rospy.Publisher(self.__full_ros_namespace + '/set_position_goal_joint',
                                                             JointState, latch = True, queue_size = 1)
        self.__set_position_cartesian_pub = rospy.Publisher(self.__full_ros_namespace
                                                            + '/set_position_cartesian',
                                                            Pose, latch = True, queue_size = 1)
        self.__set_position_goal_cartesian_pub = rospy.Publisher(self.__full_ros_namespace
                                                                 + '/set_position_goal_cartesian',
                                                                 Pose, latch = True, queue_size = 1)
        self.__set_position_jaw_pub = rospy.Publisher(self.__full_ros_namespace
                                                      + '/set_position_jaw',
                                                      JointState, latch = True, queue_size = 1)
        self.__set_position_goal_jaw_pub = rospy.Publisher(self.__full_ros_namespace
                                                           + '/set_position_goal_jaw',
                                                           JointState, latch = True, queue_size = 1)

        self.__pub_list = [self.__set_position_joint_pub,
                           self.__set_position_goal_joint_pub,
                           self.__set_position_cartesian_pub,
                           self.__set_position_goal_cartesian_pub,
                           self.__set_position_jaw_pub,
                           self.__set_position_goal_jaw_pub]


        self.__sub_list = [rospy.Subscriber(self.__full_ros_namespace + '/goal_reached',
                                          Bool, self.__goal_reached_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_current',
                                          PoseStamped, self.__position_cartesian_current_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/io/joint_position',
                                            JointState, self.__position_joint_current_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/state_jaw_current',
                                            JointState, self.__position_jaw_current_cb)]

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('dvrkArm_node', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')


    """
    Callback function
    """
    def __goal_reached_cb(self, data):
        """Callback for the goal reached.
        """
        self.__goal_reached = data.data
        self.__goal_reached_event.set()

    def __position_cartesian_current_cb(self, data):
        """Callback for the current cartesian position.
        """
        self.__position_cartesian_current = posemath.fromMsg(data.pose)

    def __position_joint_current_cb(self, data):
        """Callback for the current joint position.
        """
        self.__position_joint_current.resize(len(data.position))
        self.__position_joint_current.flat[:] = data.position

    def __position_jaw_current_cb(self, data):
        """Callback for the current jaw position.
        """
        self.__position_jaw_current = data.position


    """
    Get States function
    """
    def get_current_pose_frame(self):
        """

        :return: PyKDL.Frame
        """
        return self.__position_cartesian_current

    def get_current_pose(self,unit='rad'):    # Unit: pos in (mm) rot in (rad) or (deg)
        """

        :param unit: 'rad' or 'deg'
        :return: Numpy.array
        """
        pos,rot = self.PyKDLFrame_to_NumpyArray(self.__position_cartesian_current)
        if unit == 'deg':
            rot = self.rad_to_deg(rot)
        return pos,rot

    def get_current_joint(self, unit='rad'):
        """

        :param unit: 'rad' or 'deg'
        :return: List
        """
        joint = self.__position_joint_current
        if unit == 'deg':
            joint = self.rad_to_deg(self.__position_joint_current)
            joint[2] = self.__position_joint_current[2]
        return joint

    def get_current_jaw(self,unit):
        """

        :param unit: 'rad' or 'deg'
        :return: Numpy.float64
        """
        jaw = np.float64(self.__position_jaw_current)
        if unit == "deg":
            jaw = self.rad_to_deg(self.__position_jaw_current)
        return jaw


    """
    Set States function
    """
    def set_pose_frame(self, frame):
        """

        :param frame: PyKDL.Frame
        """
        msg = posemath.toMsg(frame)
        return self.__set_position_goal_cartesian_publish_and_wait(msg)

    def set_pose(self, pos, rot, unit='rad', wait_callback=True):
        """

        :param pos_des: position array [x,y,z]
        :param rot_des: rotation array [Z,Y,X euler angle]
        :param unit: 'rad' or 'deg'
        :param wait_callback: True or False
        """
        if unit == 'deg':
            rot = self.deg_to_rad(rot)
        # set in position cartesian mode
        frame = self.NumpyArraytoPyKDLFrame(pos, rot)
        msg = posemath.toMsg(frame)
        # go to that position by goal
        if wait_callback:
            return self.__set_position_goal_cartesian_publish_and_wait(msg)
        else:
            self.__set_position_goal_cartesian_pub.publish(msg)
            return True

    def __set_position_goal_cartesian_publish_and_wait(self, msg):
        """

        :param msg: pose
        :returns: returns true if the goal is reached
        """
        self.__goal_reached_event.clear()
        # the goal is originally not reached
        self.__goal_reached = False
        # recursively call this function until end is reached
        self.__set_position_goal_cartesian_pub.publish(msg)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        return True

    def set_joint(self, joint, unit='rad', wait_callback=True):
        """

        :param joint: joint array [j1, ..., j6]
        :param unit: 'rad', or 'deg'
        :param wait_callback: True or False
        """
        if unit == 'deg':
            joint = self.deg_to_rad(joint)
        msg = JointState()
        msg.position = joint
        if wait_callback:
            return self.__set_position_goal_joint_publish_and_wait(msg)
        else:
            self.__set_position_goal_joint_pub.publish(msg)
            return True

    def __set_position_goal_joint_publish_and_wait(self, msg):
        """

        :param msg: there is only one parameter, msg which tells us what the ending position is
        :returns: whether or not you have successfully moved by goal or not
        """
        self.__goal_reached_event.clear()
        # the goal is originally not reached
        self.__goal_reached = False
        # recursively call this function until end is reached
        self.__set_position_goal_joint_pub.publish(msg)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        return True

    def set_jaw(self, jaw, unit='rad', wait_callback=True):
        """

        :param jaw: jaw angle
        :param unit: 'rad' or 'deg'
        :param wait_callback: True or False
        """
        if unit == 'deg':
            jaw = self.deg_to_rad(jaw)
        msg = JointState()
        msg.position = [jaw]
        if wait_callback:
            return self.__set_position_goal_jaw_publish_and_wait(msg)
        else:
            self.__set_position_goal_jaw_pub.publish(msg)
            return True

    def __set_position_goal_jaw_publish_and_wait(self,msg):
        """

        :param msg:
        :return: whether or not you have successfully moved by goal or not
        """
        self.__goal_reached_event.clear()
        # the goal is originally not reached
        self.__goal_reached = False
        # recursively call this function until end is reached
        self.__set_position_goal_jaw_pub.publish(msg)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        return True


    """
    Conversion function
    """
    def rad_to_deg(self,rad):
        return np.array(rad) *180./np.pi

    def deg_to_rad(self,deg):
        return np.array(deg) *np.pi/180.

    def PyKDLFrame_to_NumpyArray(self,frame):
        pos = np.array([frame.p[0], frame.p[1], frame.p[2]])
        rz, ry, rx = self.__position_cartesian_current.M.GetEulerZYX()
        rot = np.array([np.pi/2, 0, np.pi]) - np.array([rz, ry, rx])
        return pos,rot

    def NumpyArraytoPyKDLFrame(self,pos,rot):
        px, py, pz = pos
        rz, ry, rx = np.array([np.pi / 2, 0, -np.pi]) - np.array(rot)
        return PyKDL.Frame(PyKDL.Rotation.EulerZYX(rz, ry, rx), PyKDL.Vector(px, py, pz))

if __name__ == "__main__":
    p = dvrkArm('/PSM1')
    # pos_des = [0.0, 0.0, -0.15]  # Position (m)
    # rot_des = [0, 0, 0]  # Euler angle ZYX (or roll-pitch-yaw)
    # jaw_des = [0]
    # p.set_pose(pos_des, rot_des, 'deg')
    joint = [0, 0, 0.15, 0, 0, 0]
    # ps.set_joint(joint)
    jaw = 0
    p.set_jaw(jaw, 'deg')
    # print p.get_current_pose_frame()
    # print p.get_current_pose('deg')
    # print p.get_current_joint('deg')
    # print p.get_current_jaw('deg')