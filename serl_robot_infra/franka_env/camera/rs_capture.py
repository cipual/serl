import numpy as np
import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API


class RSCapture:
    def get_device_serial_numbers(self):
        devices = rs.context().devices
        return [d.get_info(rs.camera_info.serial_number) for d in devices]

    def __init__(self, name, serial_number, dim=(640, 480), fps=15, depth=False):
        self.name = name
        assert serial_number in self.get_device_serial_numbers()
        self.serial_number = serial_number
        self.color_dist_coeffs = np.zeros(6)
        self.depth_dist_coeffs = np.zeros(6)
        self.color_intrinsics = None
        self.depth_intrinsics = None
        self.width = dim[0]
        self.height = dim[1]
        self.depth = depth
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_device(self.serial_number)
        self.cfg.enable_stream(rs.stream.color, dim[0], dim[1], rs.format.bgr8, fps)
        if self.depth:
            self.cfg.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, fps)
        self.profile = self.pipe.start(self.cfg)
        self.get_intrinsics()

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def read(self):
        frames = self.pipe.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if self.depth:
            depth_frame = aligned_frames.get_depth_frame()

        if color_frame.is_video_frame():
            image = np.asarray(color_frame.get_data())
            if self.depth and depth_frame.is_depth_frame():
                depth = np.expand_dims(np.asarray(depth_frame.get_data()), axis=2)
                return True, np.concatenate((image, depth), axis=-1)
            else:
                return True, image
        else:
            return False, None

    def close(self):
        self.pipe.stop()
        self.cfg.disable_all_streams()

    def get_intrinsics(self):
        intr_color = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        camera_matrix = np.array(
            [
                [intr_color.fx, 0, intr_color.ppx],
                [0, intr_color.fy, intr_color.ppy],
                [0,             0,              1],
            ]
        )
        dist_coeffs = np.array(intr_color.coeffs)
        self.color_intrinsics = camera_matrix
        self.color_dist_coeffs = dist_coeffs
        if self.depth:
            intr_depth = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            camera_matrix = np.array(
                [
                    [intr_depth.fx, 0, intr_depth.ppx],
                    [0, intr_depth.fy, intr_depth.ppy],
                    [0,             0,              1],
                ]
            )
            dist_coeffs = np.array(intr_depth.coeffs)
            self.depth_intrinsics = camera_matrix
            self.depth_dist_coeffs = dist_coeffs
            return self.color_intrinsics, self.color_dist_coeffs, self.depth_intrinsics, self.depth_dist_coeffs
        return self.color_intrinsics, self.color_dist_coeffs
    
    def deproject_pixel_to_point(self, depth_intrinsics, pixel, depth_value):
        point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, pixel, depth_value)
        return point_3d

"""test"""

def show_rs_capture(cam: RSCapture):
    """
    显示 RSCapture 相机采集的图像。
    按 'q' 键退出显示。
    """
    while True:
        ret, frame = cam.read()
        if not ret:
            print("未能读取图像")
            continue

        # 彩色图像处理
        color_img = frame[..., :3]
        cv2.imshow("RGB Image", color_img)

        # 如果有深度信息
        if cam.depth and frame.shape[-1] == 4:
            depth_img = frame[..., 3]
            depth_vis = cv2.convertScaleAbs(depth_img, alpha=0.03)
            depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow("Depth Image", depth_color)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    cam.close()

from cv2 import aruco
from scipy.spatial.transform import Rotation

def get_target_pose(cam: RSCapture, markerLength):
    """
    显示 ArUco 姿态（坐标轴），获取位姿变换矩阵。
    如果 isinit=True，会记录目标与 ArUco 的相对位姿。
    """
    # 相机到末端的外参
    M_camera2end = np.array([
        [-0.99976251,  0.01895704,  0.0107499,   0.03372786],
        [-0.02115898, -0.96247701, -0.27053708,  0.11196437],
        [ 0.00521795, -0.27070028,  0.96264954, -0.02858803],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # 加载字典和检测参数
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    parameters = aruco.DetectorParameters()
    assert cam.depth, "请开启深度相机"
    color_matrix, color_dist_coeffs, depth_matrix, depth_dist_coeffs = cam.get_intrinsics()

    intrinsics = rs.intrinsics()
    intrinsics.width = cam.width
    intrinsics.height = cam.height
    intrinsics.ppx = color_matrix[0, 2]
    intrinsics.ppy = color_matrix[1, 2]
    intrinsics.fx = color_matrix[0, 0]
    intrinsics.fy = color_matrix[1, 1]
    intrinsics.model = rs.distortion.none
    intrinsics.coeffs = list(color_dist_coeffs)

    while True:
        ret, frame = cam.read()
        if not ret:
            print("图像获取失败")
            continue
        print("Frame dtype:", frame.dtype, "shape:", frame.shape)

        color = frame[..., :3].astype(np.uint8)
        depth = frame[..., 3]  # 不做类型转换！

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # 检测 ArUco markers
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
            
            # 反投影为 3D 空间点坐标
            point_3d = cam.deproject_pixel_to_point(intrinsics, [u, v], depth_value)
            tvec = np.array(point_3d).squeeze()  # 相机坐标系下的位置
            aruco.drawDetectedMarkers(color, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, markerLength=markerLength,
                cameraMatrix=color_matrix, distCoeffs=color_dist_coeffs
            )

            for rvec, _ in zip(rvecs, tvecs):
                rvec = rvec[0]
                print(f'rvec: {rvec}, tvec1: {tvec}, tvec2: {_}')
                # 在图像中绘制坐标轴
                color = draw_axis(color, color_matrix, color_dist_coeffs, rvec, tvec, length=0.03)

        cv2.imshow("ArUco Pose", color)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.close()
    cv2.destroyAllWindows()

import cv2
import numpy as np

def draw_axis(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.05):
    # 定义坐标轴的三个端点（相机坐标系下）
    axis = np.float32([
        [length, 0, 0],    # X轴 — 红色
        [0, length, 0],    # Y轴 — 绿色
        [0, 0, length]     # Z轴 — 蓝色
    ]).reshape(-1, 3)

    # 投影到图像坐标系
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)

    corner = tuple(np.int32(cv2.projectPoints(np.zeros((1, 3)), rvec, tvec, camera_matrix, dist_coeffs)[0].reshape(-1, 2))[0])
    imgpts = np.int32(imgpts).reshape(-1, 2)

    # 画三条轴线
    img = cv2.line(img, corner, tuple(imgpts[0]), (0, 0, 255), 2)  # X - 红
    img = cv2.line(img, corner, tuple(imgpts[1]), (0, 255, 0), 2)  # Y - 绿
    img = cv2.line(img, corner, tuple(imgpts[2]), (255, 0, 0), 2)  # Z - 蓝

    return img



import cv2
if __name__ == "__main__":
    wrist_1 = RSCapture(name="wrist_1", serial_number="008222071923",depth=True)
    # wrist_2 = RSCapture(name="wrist_2", serial_number="250122073121", depth=True)
    get_target_pose(wrist_1, 0.05)
