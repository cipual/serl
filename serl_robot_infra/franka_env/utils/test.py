import numpy as np
from scipy.spatial.transform import Rotation as R
import time

def interpolate_move(currpos, goal: np.ndarray):
    """Move the robot to the goal position with linear interpolation."""
    steps = int(15)
    currpos = currpos
    if np.dot(currpos[3:], goal[3:]) < 0:
        currpos[3:] = np.negative(currpos[3:])
    path = np.linspace(currpos, goal, steps)
    np.set_printoptions(precision=7)
    for p in path:
        print(p)
        # self._send_pos_command(p)
        time.sleep(1 / 10)
    # self._update_currpos()

# 定义 XYZ 欧拉角 (单位：度)
euler_angles = [180, 11, -90]

# 转换为四元数
quaternion = R.from_euler('xyz', euler_angles, degrees=True).as_quat()

print(quaternion)  # 输出格式: [qx, qy, qz, qw]


q = [-0.7047357123077173, 0.7076248543435275, 0.021466365491943702, -0.04640944371571591]
rpy = R.from_quat(quaternion).as_euler("xyz", degrees=True)

print(rpy)

if __name__ == "__main__":
    currpos = [-0.603, -0.094, 0.284, -0.70678261,  0.70558068,  0.04643172, -0.02140896]
    goal = [-0.603, -0.094, 0.284, 0.7038514,  -0.7038514,  -0.06777318, -0.06777318]
    interpolate_move(currpos, goal)