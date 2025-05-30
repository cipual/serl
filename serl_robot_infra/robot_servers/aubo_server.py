"""
This file starts a control server running on the real time PC connected to the franka robot.
In a screen run `python franka_server.py`
"""

"""
---------franka_state_controller/franka_states----------
	q:	 Measured joint position.
    dq:  Measured joint velocity.

"""



from flask import Flask, request, jsonify
import numpy as np
import rospy
import time
import subprocess
from scipy.spatial.transform import Rotation as R
from absl import app, flags

import sys
sys.path.append('/home/star/serl/franka_ws/devel/lib/python3/dist-packages')
sys.path.append('/home/star/serl/serl_robot_infra')
sys.path.append('/home/star/catkin_ws/devel/lib/python3/dist-packages')

from franka_msgs.msg import ErrorRecoveryActionGoal, FrankaState
from serl_franka_controllers.msg import ZeroJacobian
import geometry_msgs.msg as geom_msg
from std_msgs.msg import Float64MultiArray
from dynamic_reconfigure.client import Client as ReconfClient
from my_aubo_controller.srv import Move

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "robot_ip", "192.168.1.25", "IP address of the franka robot's controller box"
)
flags.DEFINE_string(
    "gripper_ip", "192.168.1.114", "IP address of the robotiq gripper if being used"
)
flags.DEFINE_string(
    "gripper_type", "Aubo", "Type of gripper to use: Robotiq, Franka, or None, or Aubo"
)
flags.DEFINE_list(
    "reset_joint_target",
    [-0.4, -0.1215, 0.5476, 7.07106781e-01, -7.07106781e-01, 0, 0],
    "Target joint angles for the robot to reset to",
)


class AuboServer:
    """Handles the starting and stopping of the impedance controller
    (as well as backup) joint recovery policy."""

    def __init__(self, robot_ip, gripper_type, ros_pkg_name, reset_joint_target):
        self.robot_ip = robot_ip
        self.ros_pkg_name = ros_pkg_name
        self.reset_joint_target = reset_joint_target
        self.gripper_type = gripper_type
        self.dq = np.array(np.zeros(6))
        self.q = np.array(np.zeros(6))
        self.force = np.array(np.zeros(3))
        self.torque = np.array(np.zeros(3))
        self.jacobian = np.zeros((6, 7))
        self.vel = np.zeros(6)

        self.eepub = rospy.Publisher(
            "/aubo_move_controller/target_pose",
            geom_msg.PoseStamped,
            queue_size=10,
        )
        self.resetpub = rospy.Publisher(
            "/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=1
        )
        self.jacobian_sub = rospy.Subscriber(
            "/cartesian_impedance_controller/franka_jacobian",
            ZeroJacobian,
            self._set_jacobian,
        )
        self.state_sub = rospy.Subscriber(
            "aubo_state_controller/aubo_states", geom_msg.Pose, self._set_currpos
        )
        self.force_sub = rospy.Subscriber(
            "/SBchuan/force_pub", Float64MultiArray, self._set_currpos
        )
        rospy.wait_for_service('Move')
        self.movereq = rospy.ServiceProxy('Move', Move)
    def start_impedance(self):
        """Launches the impedance controller"""
        self.imp = subprocess.Popen(
            [
                "roslaunch",
                self.ros_pkg_name,
                "impedance.launch",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Franka' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(5)

    def stop_impedance(self):
        """Stops the impedance controller"""
        self.imp.terminate()
        time.sleep(1)

    def reset_joint(self):
        """Resets Joints (needed after running for hours)"""
        # First Stop impedance
        try:
            self.stop_impedance()
        except:
            print("impedance Not Running")
        time.sleep(3)

        # Launch joint controller reset
        # set rosparm with rospkg
        # rosparam set /target_joint_positions '[q1, q2, q3, q4, q5, q6]'
        rospy.set_param("/target_joint_positions", self.reset_joint_target)

        self.joint_controller = subprocess.Popen(
            [
                "roslaunch",
                self.ros_pkg_name,
                "joint.launch",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Aubo' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(1)
        print("RUNNING JOINT RESET")

        # Wait until target joint angles are reached
        count = 0
        time.sleep(1)
        while not np.allclose(
            np.array(self.reset_joint_target) - np.array(self.q),
            0,
            atol=1e-2,
            rtol=1e-2,
        ):
            time.sleep(1)
            count += 1
            if count > 30:
                print("joint reset TIMEOUT")
                break

        # Stop joint controller
        print("RESET DONE")
        self.joint_controller.terminate()
        time.sleep(1)
        print("KILLED JOINT RESET", self.pos)

        # Restart impedece controller
        self.start_impedance()
        print("impedance STARTED")

    """rostopic publisher"""
    # def move(self, pose: list):
    #     """Moves to a pose: [x, y, z, qx, qy, qz, qw]"""
    #     assert len(pose) == 7
    #     msg = geom_msg.PoseStamped()
    #     msg.header.frame_id = "0"
    #     msg.header.stamp = rospy.Time.now()
    #     msg.pose.position = geom_msg.Point(pose[0], pose[1], pose[2])
    #     msg.pose.orientation = geom_msg.Quaternion(pose[3], pose[4], pose[5], pose[6])
    #     self.eepub.publish(msg)
    
    """rossrv client"""
    def move(self, pose: list):
        """Moves to a pose: [x, y, z, qx, qy, qz, qw]"""
        assert len(pose) == 7
        x, y, z, qx, qy, qz, qw = pose[:]
        resp = self.movereq(x, y, z, qx, qy, qz, qw)
        while not resp.success:
            pass

    def _set_currpos(self, msg):
        if isinstance(msg, geom_msg.Pose):
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            self.pos = np.array([x, y, z, qx ,qy, qz, qw])
        elif isinstance(msg, Float64MultiArray):
            self.force = np.array(msg.data[:3])
            self.torque = np.array(msg.data[3:])
        self.dq = np.array(np.zeros(6))
        self.q = np.array(np.zeros(6))
        self.jacobian = np.zeros((6, 7))
        # print(f"pos:{self.pos}")
        try:
            self.vel = self.jacobian @ self.dq
        except:
            self.vel = np.zeros(6)
            # rospy.logwarn(
            #     "Jacobian not set, end-effector velocity temporarily not available"
            # )

    def _set_jacobian(self, msg):
        jacobian = np.array(list(msg.zero_jacobian)).reshape((6, 7), order="F")
        self.jacobian = jacobian


###############################################################################


def main(_):
    ROS_PKG_NAME = "my_aubo_controller"

    ROBOT_IP = FLAGS.robot_ip
    GRIPPER_IP = FLAGS.gripper_ip
    GRIPPER_TYPE = FLAGS.gripper_type
    RESET_JOINT_TARGET = FLAGS.reset_joint_target

    webapp = Flask(__name__)

    try:
        roscore = subprocess.Popen("roscore")
        time.sleep(1)
    except Exception as e:
        raise Exception("roscore not running", e)

    # Start ros node
    rospy.init_node("aubo_control_api")

    if GRIPPER_TYPE == "Robotiq":
        from robot_servers.robotiq_gripper_server import RobotiqGripperServer

        gripper_server = RobotiqGripperServer(gripper_ip=GRIPPER_IP)
    elif GRIPPER_TYPE == "Aubo":
        from robot_servers.aubo_gripper_server import AuboGripperServer

        gripper_server = AuboGripperServer()
    elif GRIPPER_TYPE == "None":
        pass
    else:
        raise NotImplementedError("Gripper Type Not Implemented")

    """Starts impedance controller"""
    robot_server = AuboServer(
        robot_ip=ROBOT_IP,
        gripper_type=GRIPPER_TYPE,
        ros_pkg_name=ROS_PKG_NAME,
        reset_joint_target=RESET_JOINT_TARGET,
    )
    # robot_server.start_impedance()

    # Route for Starting impedance
    @webapp.route("/startimp", methods=["POST"])
    def start_impedance():
        robot_server.start_impedance()
        return "Started impedance"

    # Route for Stopping impedance
    @webapp.route("/stopimp", methods=["POST"])
    def stop_impedance():
        robot_server.stop_impedance()
        return "Stopped impedance"

    # Route for Getting Pose
    @webapp.route("/getpos", methods=["POST"])
    def get_pos():
        return jsonify({"pose": np.array(robot_server.pos).tolist()})
    
# curl -X POST http://127.0.0.1:5000/getpos_euler
    @webapp.route("/getpos_euler", methods=["POST"])
    def get_pos_euler():
        r = R.from_quat(robot_server.pos[3:])
        euler = r.as_euler("xyz")
        return jsonify({"pose": np.concatenate([robot_server.pos[:3], euler]).tolist()})

# curl -X POST http://localhost:5000/getvel
    @webapp.route("/getvel", methods=["POST"])
    def get_vel():
        return jsonify({"vel": np.array(robot_server.vel).tolist()})

# curl -X POST http://localhost:5000/getforce
    @webapp.route("/getforce", methods=["POST"])
    def get_force():
        return jsonify({"force": np.array(robot_server.force).tolist()})

# curl -X POST http://localhost:5000/gettorque
    @webapp.route("/gettorque", methods=["POST"])
    def get_torque():
        return jsonify({"torque": np.array(robot_server.torque).tolist()})

# curl -X POST http://localhost:5000/getq
    @webapp.route("/getq", methods=["POST"])
    def get_q():
        return jsonify({"q": np.array(robot_server.q).tolist()})

# curl -X POST http://localhost:5000/getdq
    @webapp.route("/getdq", methods=["POST"])
    def get_dq():
        return jsonify({"dq": np.array(robot_server.dq).tolist()})

# curl -X POST http://localhost:5000/getjacobian
    @webapp.route("/getjacobian", methods=["POST"])
    def get_jacobian():
        return jsonify({"jacobian": np.array(robot_server.jacobian).tolist()})

    # Route for getting gripper distance
    @webapp.route("/get_gripper", methods=["POST"])
    def get_gripper():
        return jsonify({"gripper": gripper_server.gripper_pos})

    # Route for Running Joint Reset
    @webapp.route("/jointreset", methods=["POST"])
    def joint_reset():
        robot_server.reset_joint()
        return "Reset Joint"

    # Route for Activating the Gripper
    @webapp.route("/activate_gripper", methods=["POST"])
    def activate_gripper():
        print("activate gripper")
        gripper_server.activate_gripper()
        return "Activated"

    # Route for Resetting the Gripper. It will reset and activate the gripper
    @webapp.route("/reset_gripper", methods=["POST"])
    def reset_gripper():
        print("reset gripper")
        gripper_server.reset_gripper()
        return "Reset"

# curl -X POST http://localhost:5000/open_gripper
    # Route for Opening the Gripper
    @webapp.route("/open_gripper", methods=["POST"])
    def open():
        print("open")
        gripper_server.open()
        return "Opened"
    
# curl -X POST http://localhost:5000/close_gripper
    # Route for Closing the Gripper
    @webapp.route("/close_gripper", methods=["POST"])
    def close():
        print("close")
        gripper_server.close()
        return "Closed"

    # Route for moving the gripper
    @webapp.route("/move_gripper", methods=["POST"])
    def move_gripper():
        gripper_pos = request.json
        pos = np.clip(int(gripper_pos["gripper_pos"]), 0, 1000)  # 0-1000
        print(f"move gripper to {pos}")
        gripper_server.move(pos)
        return "Moved Gripper"

# curl -X POST http://localhost:5000/pose -H "Content-Type: application/json" -d '{"arr": [x, y, z, qx, qy, qz, qw]}'
    # Route for Sending a pose command
    @webapp.route("/pose", methods=["POST"])
    def pose():
        pos = np.array(request.json["arr"])
        print("Moving to", pos)
        robot_server.move(pos)
        return "Moved"

# curl -X POST http://localhost:5000/getstate
    # Route for getting all state information
    @webapp.route("/getstate", methods=["POST"])
    def get_state():
        return jsonify(
            {
                "pose": np.array(robot_server.pos).tolist(),
                "vel": np.array(robot_server.vel).tolist(),
                "force": np.array(robot_server.force).tolist(),
                "torque": np.array(robot_server.torque).tolist(),
                "q": np.array(robot_server.q).tolist(),
                "dq": np.array(robot_server.dq).tolist(),
                "jacobian": np.array(robot_server.jacobian).tolist(),
                "gripper_pos": gripper_server.gripper_pos,
            }
        )

    webapp.run(host="0.0.0.0")


if __name__ == "__main__":
    app.run(main)
