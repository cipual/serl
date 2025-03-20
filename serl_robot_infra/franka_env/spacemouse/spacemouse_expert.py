import threading
import pyspacemouse
import numpy as np
from typing import Tuple
import sys
sys.path.append('/home/star/catkin_ws')
from collections import deque
import time
import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from pynput import keyboard

ACTION_SCALE = np.array([0.02, 0.1, 1])
INTERVENE_STEPS = 20



class SpaceMouseExpert:
    """
    This class provides an interface to the SpaceMouse.
    It continuously reads the SpaceMouse state and provide
    a "get_action" method to get the latest action and button state.
    """

    def __init__(self):
        pyspacemouse.open()

        self.state_lock = threading.Lock()
        self.latest_data = {"action": np.zeros(6), "buttons": [0, 0]}
        # Start a thread to continuously read the SpaceMouse state
        self.thread = threading.Thread(target=self._read_spacemouse)
        self.thread.daemon = True
        self.thread.start()

    def _read_spacemouse(self):
        while True:
            state = pyspacemouse.read()
            with self.state_lock:
                self.latest_data["action"] = np.array(
                    [-state.y, state.x, state.z, -state.roll, -state.pitch, -state.yaw]
                )  # spacemouse axis matched with robot base frame
                self.latest_data["buttons"] = state.buttons

    def get_action(self) -> Tuple[np.ndarray, list]:
        """Returns the latest action and button state of the SpaceMouse."""
        with self.state_lock:
            return self.latest_data["action"], self.latest_data["buttons"]


class AuboSoftExpert:
    def __init__(self, isrecord):
        rospy.init_node("action_collector", anonymous=True)
        
        # 创建客户端，连接到机械臂控制服务
        rospy.wait_for_service('control_arm')
        self.control_arm = rospy.ServiceProxy('control_arm', SetBool)

        rospy.wait_for_service('begin_receive')
        self.begin_receive = rospy.ServiceProxy('begin_receive', SetBool)

        # 订阅节点2的action消息
        self.subscriber = rospy.Subscriber("/pub_demo/delta_action", Float64MultiArray, self.callback)
        
        # 存储接收到的 action
        self.actions = deque()
        self.last_received_time = time.time()
        self.received_count = 0
        self.current_action = []
        self.last_action = []
        self.OK =False #开始接受信息的标志
        self.intervene_steps = INTERVENE_STEPS # -1:干预到设定目标点
        self.p_pressed = False
        self.record_mode = isrecord

        def on_press(key):
            """ 按键按下事件 """
            try:
                if key.char == 'p':
                    self.p_pressed = True
            except AttributeError:
                pass

        # 键盘监听
        self.listener = keyboard.Listener(on_press=on_press)
        self.listener.start()  # 非阻塞启动监听线程


    def __del__(self):
        """对象销毁时停止键盘监听"""
        self.listener.stop()

    def callback(self, msg):
        """接收 action 并存入队列"""
        action = list(msg.data)
        self.last_action = self.current_action
        self.current_action = action  
        if self.OK: # 只记录不同信息
            action[:3] /= ACTION_SCALE[0]
            action[3:] /= ACTION_SCALE[1]
            rospy.loginfo(f"action:{action}")
            self.actions.append(action)
            self.received_count += 1
            self.last_received_time = time.time()  # 记录最后接收到 action 的时间

    def create_action(self):
        """发送指令并监听 action 消息"""
        rospy.loginfo("发送启动指令给节点1")
        
        try:
            # 发送启动指令
            msg_response = self.begin_receive(True)
            rospy.loginfo(f"开始接收动作增量：success={msg_response.success}, message={msg_response.message}")
           
            arm_response = self.control_arm(True)
            rospy.loginfo(f"机械臂启动响应: success={arm_response.success}, message={arm_response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"调用机械臂控制服务失败: {e}")
            return []

        rate = rospy.Rate(10)  # 10Hz 监听频率
        
        while not rospy.is_shutdown():
            current_time = time.time()

            # 如果超时 2 秒没有新数据，认为运动结束
            if (current_time - self.last_received_time > 2):
                rospy.loginfo("节点2的 action 停止更新，发送停止指令给节点1")
                try:
                    msg_response = self.begin_receive(False)
                    rospy.loginfo(f"关闭接收动作增量：success={msg_response.success}, message={msg_response.message}")
                    
                    response = self.control_arm(False)
                    rospy.loginfo(f"机械臂停止响应: success={response.success}, message={response.message}")
                    self.OK = False
                except rospy.ServiceException as e:
                    rospy.logerr(f"调用机械臂控制服务失败: {e}")              
                break
            
            rate.sleep()
        
        return list(self.actions)
    
    def get_action(self):
        try:
            if self.intervene_steps > 0:
                self.intervene_steps -= 1
            return self.actions.popleft()
        except IndexError:
            # print("no intervene action found!")
            return list(np.zeros(6,))

if __name__ == "__main__":
    ABexpert = AuboSoftExpert()
    ABexpert.OK = True
    actions = ABexpert.create_action()
    ABexpert.OK = False
    print("收集到的动作数据:", actions)
    print(f"数据长度为：{len(actions)}")
    print(f"第一个action：{ABexpert.get_action()}")
