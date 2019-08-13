from sensor_msgs.msg import JointState

a = JointState(position=list([1,2,3]))
# a.position = [1,2,3,4,5,6,7,8,9]

# JointState(name=self._ik.joints, position=list(result[1]))
print a
