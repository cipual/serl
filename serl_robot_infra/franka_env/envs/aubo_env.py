"""Gym Interface for Franka"""
import numpy as np
import gym
import cv2
import cv2.aruco as aruco
import copy
from scipy.spatial.transform import Rotation
import time
import requests
import queue
import threading
from datetime import datetime
from collections import OrderedDict
from typing import Dict
import pyrealsense2 as rs
from franka_env.camera.video_capture import VideoCapture
from franka_env.camera.rs_capture import RSCapture
from franka_env.utils.rotations import euler_2_quat, quat_2_euler


class ImageDisplayer(threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.daemon = True  # make this a daemon thread

    def run(self):
        while True:
            img_array = self.queue.get()  # retrieve an image from the queue
            if img_array is None:  # None is our signal to exit
                break

            frame = np.concatenate(
                [v for k, v in img_array.items() if "full" not in k], axis=0
            )

            cv2.imshow("RealSense Cameras", frame)
            cv2.waitKey(1)


##############################################################################


class DefaultEnvConfig:
    """Default configuration for FrankaEnv. Fill in the values below."""

    SERVER_URL: str = "http://127.0.0.1:5000/"
    REALSENSE_CAMERAS: Dict = {
        "wrist_1": "008222071923",
        "wrist_2": "250122073121",
    }
    TARGET_POSE: np.ndarray = np.zeros((6,))
    REWARD_THRESHOLD: np.ndarray = np.zeros((6,))
    ACTION_SCALE = np.zeros((3,))
    RESET_POSE = np.zeros((6,))
    RANDOM_RESET = (False,)
    RANDOM_XY_RANGE = (0.0,)
    RANDOM_RZ_RANGE = (0.0,)
    ABS_POSE_LIMIT_HIGH = np.zeros((6,))
    ABS_POSE_LIMIT_LOW = np.zeros((6,))
    COMPLIANCE_PARAM: Dict[str, float] = {}
    PRECISION_PARAM: Dict[str, float] = {}
    BINARY_GRIPPER_THREASHOLD: float = 0.5
    APPLY_GRIPPER_PENALTY: bool = True
    GRIPPER_PENALTY: float = 0.1


##############################################################################


class AuboEnv(gym.Env):
    def __init__(
        self,
        hz=10,
        fake_env=False,
        save_video=True,
        config: DefaultEnvConfig = None,
        max_episode_length=200,
    ):
        if config:
            print(config.ACTION_SCALE)
        self.action_scale = config.ACTION_SCALE
        self._TARGET_POSE = config.TARGET_POSE
        self._REWARD_THRESHOLD = config.REWARD_THRESHOLD
        self.stop_threshold = np.array([0.0001, 0.0001, 0.0001, 0.0001, 0.001, 0.001, 0.001])
        self.url = config.SERVER_URL
        self.config = config
        self.max_episode_length = max_episode_length

        # convert last 3 elements from euler to quat, from size (6,) to (7,)
        self.resetpos = np.concatenate(
            [config.RESET_POSE[:3], euler_2_quat(config.RESET_POSE[3:])]
        )

        self.currpos = self.resetpos.copy()
        self.lastpose = np.zeros((7,))
        self.currvel = np.zeros((6,))
        self.q = np.zeros((6,))
        self.dq = np.zeros((6,))
        self.currforce = np.zeros((3,))
        self.currtorque = np.zeros((3,))
        self.currjacobian = np.zeros((6, 7))

        self.curr_gripper_pos = 0
        self.gripper_binary_state = 0  # 0 for open, 1 for closed
        self.lastsent = time.time()
        self.randomreset = config.RANDOM_RESET
        self.random_xy_range = config.RANDOM_XY_RANGE
        self.random_rz_range = config.RANDOM_RZ_RANGE
        self.hz = hz
        self.joint_reset_cycle = 200  # reset the robot joint every 200 cycles

        if save_video:
            print("Saving videos!")
        self.save_video = save_video
        self.recording_frames = []

        # boundary box
        self.xyz_bounding_box = gym.spaces.Box(
            config.ABS_POSE_LIMIT_LOW[:3],
            config.ABS_POSE_LIMIT_HIGH[:3],
            dtype=np.float64,
        )
        self.rpy_bounding_box = gym.spaces.Box(
            config.ABS_POSE_LIMIT_LOW[3:],
            config.ABS_POSE_LIMIT_HIGH[3:],
            dtype=np.float64,
        )
        # Action/Observation Space (-1 ~ +1)
        self.action_space = gym.spaces.Box(
            np.ones((7,), dtype=np.float32) * -2,
            np.ones((7,), dtype=np.float32) * 2,
        )

        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(
                    {
                        "tcp_pose": gym.spaces.Box(
                            -np.inf, np.inf, shape=(7,)
                        ),  # xyz + quat
                        "tcp_vel": gym.spaces.Box(-np.inf, np.inf, shape=(6,)),
                        "gripper_pose": gym.spaces.Box(-1, 1, shape=(1,)),
                        "tcp_force": gym.spaces.Box(-np.inf, np.inf, shape=(3,)),
                        "tcp_torque": gym.spaces.Box(-np.inf, np.inf, shape=(3,)),
                    }
                ),
                "images": gym.spaces.Dict(
                    {
                        "wrist_1": gym.spaces.Box(
                            0, 255, shape=(128, 128, 3), dtype=np.uint8
                        ),
                        "wrist_2": gym.spaces.Box(
                            0, 255, shape=(128, 128, 3), dtype=np.uint8
                        ),
                    }
                ),
            }
        )
        self.cycle_count = 0

        if fake_env:
            return

        self.cap = None
        # ArUco marker 与 target 的相对距离(x,y,z)
        # 使用～/catkin_ws/src/my_aubo_controller/scripts/test_camera.py标定
        self.marker2target = np.array([0.0494822, -0.01097698, 0.28018171, -0.00454577, 0.0307513, 0.04513861])
        self.contactless_force = np.array([-1.8276804685592651,1.611989974975586,1.8630472421646118])
        self.contactless_torque = np.array([-0.04563365876674652,0.20035456120967865,0.01827717199921608])
        self.init_cameras(config.REALSENSE_CAMERAS)
        self.img_queue = queue.Queue()
        self.displayer = ImageDisplayer(self.img_queue)
        self.displayer.start()
        print("Initialized Aubo")

    def clip_safety_box(self, pose: np.ndarray) -> np.ndarray:
        """Clip the pose to be within the safety box."""
        pose[:3] = np.clip(
            pose[:3], self.xyz_bounding_box.low, self.xyz_bounding_box.high
        )
        euler = Rotation.from_quat(pose[3:]).as_euler("xyz")

        # Clip first euler angle separately due to discontinuity from pi to -pi
        # sign = np.sign(euler[0])
        euler[0] = (
            np.clip(
                euler[0],
                self.rpy_bounding_box.low[0],
                self.rpy_bounding_box.high[0],
            )
        )

        euler[1:] = np.clip(
            euler[1:], self.rpy_bounding_box.low[1:], self.rpy_bounding_box.high[1:]
        )
        pose[3:] = Rotation.from_euler("xyz", euler).as_quat()

        return pose

    def step(self, action_tuple: np.ndarray) -> tuple:
        """standard gym step function."""
        action, replaced = action_tuple
        start_time = time.time()
        #record demo的时候delay避免惯性误差
        if replaced:
            time.sleep(0.2)
        else:
            action = np.clip(action, self.action_space.low, self.action_space.high)
        xyz_delta = action[:3]
        self._update_currpos()
        self.nextpos = self.currpos.copy()
        if np.any(np.abs(self.contactless_force[:2] - self.currforce[:2]) > 1.5) or self.currforce[2] < 0.5:
            self.nextpos[2] += 0.010 #力过大向上1cm 
        self.nextpos[:3] = self.nextpos[:3] + xyz_delta * self.action_scale[0]

        # GET ORIENTATION FROM ACTION
        self.nextpos[3:] = (
            Rotation.from_euler("xyz", action[3:6] * self.action_scale[1])
            * Rotation.from_quat(self.currpos[3:])
        ).as_quat()

        gripper_action = action[6] * self.action_scale[2]

        gripper_action_effective = self._send_gripper_command(gripper_action)
        self._send_pos_command(self.clip_safety_box(self.nextpos))

        self.curr_path_length += 1
        dt = time.time() - start_time
        time.sleep(max(0, (1.0 / self.hz) - dt))

        self._update_currpos()
        ob = self._get_obs()
        #get new targetpose
        # self.get_target_pose()
        reward = self.compute_reward(ob, gripper_action_effective)
        done = self.curr_path_length >= self.max_episode_length or reward == 1
        return ob, reward, done, False, {}

    def compute_reward(self, obs, gripper_action_effective) -> bool:
        """We are using a sparse reward function."""
        current_pose = obs["state"]["tcp_pose"]
        # convert from quat to euler first
        euler_angles = quat_2_euler(current_pose[3:])
        # euler_angles = np.abs(euler_angles)
        current_pose = np.hstack([current_pose[:3], euler_angles])
        delta = np.abs(current_pose - self._TARGET_POSE)
        delta[3:] = (delta[3:] + np.pi) % (2 * np.pi) - np.pi #转换成-pi - pi
        delta[3:] = np.abs(delta[3:])
        if np.all(delta < self._REWARD_THRESHOLD):
            print('Goal!')
            print(f'pose:{current_pose}')
            reward = 1
        else:
            # print(f'Goal not reached, the difference is {delta}, the desired threshold is {self._REWARD_THRESHOLD}')
            reward = 0

        if self.config.APPLY_GRIPPER_PENALTY and gripper_action_effective:
            reward -= self.config.GRIPPER_PENALTY

        return reward

    def crop_image(self, name, image) -> np.ndarray:
        """Crop realsense images to be a square."""
        if name == "wrist_1":
            return image[:, 80:560, :]
        elif name == "wrist_2":
            return image[:, 80:560, :]
        else:
            return ValueError(f"Camera {name} not recognized in cropping")

    def get_im(self) -> Dict[str, np.ndarray]:
        """Get images from the realsense cameras."""
        images = {}
        display_images = {}
        for key, cap in self.cap.items():
            try:
                frame = cap.read()
                rgb = frame[..., :3].astype(np.uint8)
                cropped_rgb = self.crop_image(key, rgb)
                resized = cv2.resize(
                    cropped_rgb, self.observation_space["images"][key].shape[:2][::-1]
                )
                images[key] = resized[..., ::-1]
                display_images[key] = resized
                display_images[key + "_full"] = cropped_rgb
            except queue.Empty:
                input(
                    f"{key} camera frozen. Check connect, then press enter to relaunch..."
                )
                cap.close()
                self.init_cameras(self.config.REALSENSE_CAMERAS)
                return self.get_im()

        self.recording_frames.append(
            np.concatenate([display_images[f"{k}_full"] for k in self.cap], axis=0)
        )
        self.img_queue.put(display_images)
        return images

    def get_target_pose(self):
        """
        Get the target pose from the environment.
        """
        # wrist_1相机的外参
        wrist_1 = self.cap["wrist_1"]
        wrist_1.extrinsics = np.array([
            [-0.99974713,  0.0191012,   0.01186678,  0.02983112],
            [-0.02157909, -0.96336774, -0.26731431,  0.10141795],
            [ 0.00632605, -0.26750279,  0.96353632,  0.03283929],
            [ 0.0,         0.0,         0.0,         1.0]
            ])
        M_camera2end = wrist_1.extrinsics
        # 加载 ArUco 字典
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        parameters = aruco.DetectorParameters()
        color_matrix, color_dist_coeffs, depth_matrix, depth_dist_coeffs = wrist_1.get_intricsics()
        # 读取图像
        frame = wrist_1.read()
        color = frame[..., :3].astype(np.uint8)
        depth = frame[..., 3]  # 不做类型转换！
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        # 检测 ArUco 标记
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            # 取第一个 marker 的四个角点
            marker_corners = corners[0][0]
            center_pixel = np.mean(marker_corners, axis=0).astype(int)
            u, v = center_pixel

            # 获取深度值（以米为单位）
            depth_value = depth[v, u] * 0.001  # 深度值通常是以毫米为单位，乘0.001转换为米
            if depth_value == 0:
                print("Depth at ArUco center is zero.")
                return None
            intrinsics = rs.intrinsics()
            intrinsics.width = wrist_1.cap.width
            intrinsics.height = wrist_1.cap.height
            intrinsics.ppx = color_matrix[0, 2]
            intrinsics.ppy = color_matrix[1, 2]
            intrinsics.fx = color_matrix[0, 0]
            intrinsics.fy = color_matrix[1, 1]
            intrinsics.model = rs.distortion.none
            intrinsics.coeffs = list(color_dist_coeffs)
            # 反投影为 3D 空间点坐标
            point_3d = wrist_1.deproject_pixel_to_point(intrinsics, [u, v], depth_value)
            tvec = np.array(point_3d)  # 相机坐标系下的位置
        
            # 估计每个 marker 的位姿
            rvecs, _, _ = aruco.estimatePoseSingleMarkers(corners, markerLength=0.05, cameraMatrix=color_matrix, distCoeffs=color_dist_coeffs)
            # 计算目标位姿
            # 检测到一个 marker
            if len(rvecs) > 0:
                rvec = rvecs[0][0]
                # print(f'rvec: {rvec}, tvec: {tvec}')
                # 将旋转向量转换为旋转矩阵
                R, _ = cv2.Rodrigues(rvec)
                # aruco到相机坐标系
                M_aruco2camera = np.eye(4)
                M_aruco2camera[:3, :3] = R
                M_aruco2camera[:3, 3] = tvec
                #aruco到末端坐标系
                M_aruco2end = M_camera2end @ M_aruco2camera
                #与末端坐标系对齐
                M_aruco2end = M_aruco2end @ np.array([[-1, 0, 0, 0],
                                                      [0, 1, 0, 0],
                                                      [0, 0, -1, 0],
                                                      [0, 0, 0, 1]])
                # 末端到base坐标系
                # 将四元数列表转换为 Rotation 对象
                rotation = Rotation.from_quat(self.currpos[3:])
                # 获取旋转矩阵
                rotation_matrix = rotation.as_matrix()               
                # 构造变换矩阵
                M_end2base = np.eye(4)
                M_end2base[:3, 3] = np.array(self.currpos[:3])  # 位置
                M_end2base[:3, :3] = rotation_matrix  # 旋转部分

                M_aruco2base = M_end2base @ M_aruco2end
                # 取出位姿
                position = M_aruco2base[:3, 3]
                rotation_matrix = M_aruco2base[:3, :3]
                # 将旋转矩阵转换为欧拉角
                rotation = Rotation.from_matrix(rotation_matrix)
                euler = rotation.as_euler('xyz', degrees=False)
                # print(f'position{position}, euler{euler}')
                position_slot = position + self.marker2target[:3]
                euler_slot = euler + self.marker2target[3:]
                # 对齐目标角度
                for i in range(3):
                    if np.sign(self._TARGET_POSE[3 + i]) != np.sign(euler_slot[i]):
                        # 直接翻转角度（对 π 做镜像）
                        if abs(euler_slot[i] - self._TARGET_POSE[3 + i]) > np.pi / 2:
                            euler_slot[i] = euler_slot[i] - np.sign(euler_slot[i]) * 2 * np.pi
                        else:
                            euler_slot[i] *= -1
                self._TARGET_POSE = np.array(position_slot.tolist() + euler_slot.tolist())
                print(f"Target Pose: {self._TARGET_POSE}")
                return M_aruco2base
            else:
                print("No markers detected")
                return None
        print("No ids detected")
        return None


    def interpolate_move(self, goal: np.ndarray, timeout: float):
        """Move the robot to the goal position with linear interpolation."""
        steps = int(timeout * self.hz)
        self._update_currpos()
        if np.dot(self.currpos[3:], goal[3:]) < 0:
            self.currpos[3:] = np.negative(self.currpos[3:])
        path = np.linspace(self.currpos, goal, steps)
        for p in path:
            self._send_pos_command(p)
            time.sleep(1 / self.hz)
        self._update_currpos()

    def go_to_rest(self, joint_reset=False):
        """
        The concrete steps to perform reset should be
        implemented each subclass for the specific task.
        Should override this method if custom reset procedure is needed.
        """
        # Change to precision mode for reset
        time.sleep(0.5)

        # Perform joint reset if needed
        if joint_reset:
            print("JOINT RESET")
            requests.post(self.url + "jointreset")
            time.sleep(0.5)

        # Perform Carteasian reset
        if self.randomreset:  # randomize reset position in xy plane
            reset_pose = self.resetpos.copy()
            reset_pose[:2] += np.random.uniform(
                -self.random_xy_range, self.random_xy_range, (2,)
            )
            euler_random = self._TARGET_POSE[3:].copy()
            euler_random[-1] += np.random.uniform(
                -self.random_rz_range, self.random_rz_range
            )
            reset_pose[3:] = euler_2_quat(euler_random)
            self.interpolate_move(reset_pose, timeout=1.5)
        else:
            reset_pose = self.resetpos.copy()
            self.interpolate_move(reset_pose, timeout=1.5)

    def reset(self, joint_reset=False, **kwargs):
        if self.save_video:
            self.save_video_recording()

        self.cycle_count += 1
        if self.cycle_count % self.joint_reset_cycle == 0:
            self.cycle_count = 0
            joint_reset = True

        self.go_to_rest(joint_reset=joint_reset)
        # self._recover()
        self.curr_path_length = 0

        self._update_currpos()
        obs = self._get_obs()

        return obs, {}

    def save_video_recording(self):
        try:
            if len(self.recording_frames):
                video_writer = cv2.VideoWriter(
                    f'/home/star/serl/examples/async_peg_insert_drq/videos/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.mp4',
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    10,
                    self.recording_frames[0].shape[:2][::-1],
                )
                for frame in self.recording_frames:
                    video_writer.write(frame)
                video_writer.release()
            self.recording_frames.clear()
        except Exception as e:
            print(f"Failed to save video: {e}")

    def init_cameras(self, name_serial_dict=None):
        """Init both wrist cameras."""
        if self.cap is not None:  # close cameras if they are already open
            self.close_cameras()

        self.cap = OrderedDict()
        for cam_name, cam_serial in name_serial_dict.items():
            enable_depth = True if cam_name == "wrist_1" else False # wrist_1 need depth
            cap = VideoCapture(
                RSCapture(name=cam_name, serial_number=cam_serial, depth=enable_depth)
            )
            self.cap[cam_name] = cap

    def close_cameras(self):
        """Close both wrist cameras."""
        try:
            for cap in self.cap.values():
                cap.close()
        except Exception as e:
            print(f"Failed to close cameras: {e}")

    def _send_pos_command(self, pos: np.ndarray):
        """Internal function to send position command to the robot."""
        arr = np.array(pos).astype(np.float32)
        data = {"arr": arr.tolist()}
        requests.post(self.url + "pose", json=data)

    def _send_gripper_command(self, pos: float, mode="binary"):
        """Internal function to send gripper command to the robot."""
        if mode == "binary":
            if (
                pos <= -self.config.BINARY_GRIPPER_THREASHOLD
                and self.gripper_binary_state == 0
            ):  # close gripper
                requests.post(self.url + "close_gripper")
                time.sleep(0.6)
                self.gripper_binary_state = 1
                return True
            elif (
                pos >= self.config.BINARY_GRIPPER_THREASHOLD
                and self.gripper_binary_state == 1
            ):  # open gripper
                requests.post(self.url + "open_gripper")
                time.sleep(0.6)
                self.gripper_binary_state = 0
                return True
            else:  # do nothing to the gripper
                return False
        elif mode == "continuous":
            raise NotImplementedError("Continuous gripper control is optional")

    def _update_currpos(self):
        """
        Internal function to get the latest state of the robot and its gripper.
        """
        pos = np.zeros((7,))
        ps = requests.post(self.url + "getstate").json()
        #wait for stopping
        while True:
            # time.sleep(0.1)
            pos[:] = np.array(ps["pose"])
            self.lastpose = self.currpos
            self.currpos = pos
            if np.all(np.abs(self.lastpose - self.currpos) < self.stop_threshold):
                break

        self.currpos[:] = np.array(ps["pose"])

        self.currvel[:] = np.array(ps["vel"])

        self.currforce[:] = np.array(ps["force"])
        self.currtorque[:] = np.array(ps["torque"])
        self.currjacobian[:] = np.reshape(np.array(ps["jacobian"]), (6, 7))

        self.q[:] = np.array(ps["q"])
        self.dq[:] = np.array(ps["dq"])

        self.curr_gripper_pos = np.array(ps["gripper_pos"])

    def _get_obs(self) -> dict:
        images = self.get_im()
        state_observation = {
            "tcp_pose": self.currpos,
            "tcp_vel": self.currvel,
            "gripper_pose": self.curr_gripper_pos,
            "tcp_force": self.currforce,
            "tcp_torque": self.currtorque,
        }
        return copy.deepcopy(dict(images=images, state=state_observation))
    