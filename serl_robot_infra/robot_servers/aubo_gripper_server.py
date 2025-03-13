import rospy
from std_msgs.msg import Bool, UInt16, Float64MultiArray
import numpy as np
import sys
sys.path.append('/home/star/serl/serl_robot_infra')
from robot_servers.gripper_server import GripperServer


class AuboGripperServer(GripperServer):
    def __init__(self):
        super().__init__()
        self.grippermovepub = rospy.Publisher(
            "/aubo_gripper/move", UInt16, queue_size=1
        )
        self.grippergrasppub = rospy.Publisher(
            "/aubo_gripper/actions", Bool, queue_size=1
        )
        self.gripper_sub = rospy.Subscriber(
            "/aubo_state_controller/joint_states", Float64MultiArray, self._update_gripper
        )

    def open(self):
        msg = Bool()
        msg.data = 0
        self.grippergrasppub.publish(msg)

    def close(self):
        msg = Bool()
        msg.data = 1
        self.grippergrasppub.publish(msg)

    def move(self, position: int):
        """Move the gripper to a specific position in range [0, 1000]"""
        msg = UInt16()
        if position >= 1000:
            position = 1000
        elif position <= 0:
            position = 0
        else:
            pass
        msg.data = position
        self.grippermovepub.publish(msg)

    def _update_gripper(self, msg):
        """internal callback to get the latest gripper position."""
        self.gripper_pos = np.sum(msg.data)
