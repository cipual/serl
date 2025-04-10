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
        if self.color_intrinsics is None:
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
        if self.depth_intrinsics is None and self.depth:
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

"""test"""
if __name__ == "__main__":
    wrist_1 = RSCapture(name="wrist_1", serial_number="008222071923")
    wrist_2 = RSCapture(name="wrist_2", serial_number="250122073121")
    print('-----wrist_1-----')
    print(wrist_1.color_intrinsics)
    print(wrist_1.color_dist_coeffs)
    print('-----wrist_2-----')
    print(wrist_2.color_intrinsics)
    print(wrist_2.color_dist_coeffs)