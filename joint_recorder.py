#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from tm_msgs.msg import FeedbackState
import math
import time

class SingleRobotStateCollector(Node):
    """
    單機械手臂資料收集器:
    - 訂閱 /joint_states 與 /feedback_states
    - 每次輸出 [j1..j6(度), io0, io1, io2]
    - fps 控制輸出速率
    """
    def __init__(self, fps: int = 30):
        super().__init__('single_robot_state_collector')

        # 節流設定
        self.interval = 1.0 / max(1, fps)
        self.last_emit = 0.0

        # 緩存
        self.joints_deg = None   # list[float] 長度6
        self.io = [0, 0, 0]      # 末端/控制箱 DO 前三個

        # QoS（感測資料）
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 訂閱 joint_states
        self.create_subscription(JointState, '/joint_states', self._on_joint, qos_sensor)
        self.get_logger().info('訂閱: /joint_states')

        # 訂閱 feedback_states
        self.create_subscription(FeedbackState, '/feedback_states', self._on_fb, 10)
        self.get_logger().info('訂閱: /feedback_states')

    # ---- Callbacks ----
    def _on_joint(self, msg: JointState):
        """接收關節角度並轉換成度數"""
        if not msg.position or len(msg.position) < 6:
            return
        
        # 取前6軸並轉成度數
        j6 = msg.position[:6]
        joints_deg = [round(math.degrees(j), 2) for j in j6]

        # 過濾全0（初始訊號或未連線）
        if all(j == 0.0 for j in joints_deg):
            return

        self.joints_deg = joints_deg
        self._try_emit()

    def _on_fb(self, msg: FeedbackState):
        """接收 IO 狀態，優先使用末端 DO"""
        ee = list(msg.ee_digital_output) if msg.ee_digital_output is not None else []
        cb = list(msg.cb_digital_output) if msg.cb_digital_output is not None else []
        src = ee if len(ee) > 0 else cb

        # 取前三個 DO 並轉為 0/1
        a = int(src[0]) if len(src) > 0 else 0
        b = int(src[1]) if len(src) > 1 else 0
        c = int(src[2]) if len(src) > 2 else 0
        self.io = [1 if a else 0, 1 if b else 0, 1 if c else 0]

        self._try_emit()

    def _try_emit(self):
        """當 joint 與 IO 都有資料，且符合 fps 間隔時輸出"""
        now = time.time()
        if self.joints_deg is None:
            return
        if now - self.last_emit < self.interval:
            return

        combined = self.joints_deg + self.io
        print(self.joints_deg)  # 例如: [j1..j6(度), io0, io1, io2]

        self.last_emit = now

def main(args=None):
    rclpy.init(args=args)
    node = SingleRobotStateCollector(fps=30)  # 預設 30 fps
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
