#!/usr/bin/env python3
import math
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from uart import UART
from one_arm_control_CPP import TMRobotController

def _clamp(v, lo, hi): return max(lo, min(hi, v))

def normalize_quat(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (0.0, 0.0, 0.0, 1.0) if n == 0 else (x/n, y/n, z/n, w/n)

def quat_to_euler_zyx_deg(qx, qy, qz, qw):
    qx, qy, qz, qw = normalize_quat(qx, qy, qz, qw)
    siny = 2.0 * (qw*qz + qx*qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny, cosy)
    sinp = 2.0 * (qw*qy - qz*qx)
    sinp = _clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)
    sinr = 2.0 * (qw*qx + qy*qz)
    cosr = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr, cosr)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def pose_to_cpp(pose):
    if hasattr(pose, "pose"):
        p = pose.pose.position
        q = pose.pose.orientation
    else:
        p = pose.position
        q = pose.orientation
    x_mm, y_mm, z_mm = p.x*1000.0, p.y*1000.0, p.z*1000.0
    rx, ry, rz = quat_to_euler_zyx_deg(q.x, q.y, q.z, q.w)
    return [round(x_mm,2), round(y_mm,2), round(z_mm,2),
            round(rx,2), round(ry,2), round(rz,2)]

class PoseAndUARTNode(Node):
    def __init__(self):
        super().__init__('pose_and_uart_node')
        self.latest_pose = None
        self.controller = None   # 由 main() 注入
        self.last_uart_time = 0.0 
        self.create_subscription(PoseStamped, '/tool_pose', self.cb_pose, 10)
        self.get_logger().info('已訂閱 /tool_pose')

        # 啟動 UART，並把回呼綁到 self.on_uart_data
        self.uart = UART("/dev/ttyUSB0", 115200, on_data=self.on_uart_data)

    def cb_pose(self, msg: PoseStamped):
        self.latest_pose = pose_to_cpp(msg)
        #self.get_logger().info(f"[Pose] {self.latest_pose}")
        #fake_val = [674.57, -155.37, 697.42, 175.11, -0.7, 90.13]
        #self.controller.append_tcp(fake_val)

    def on_uart_data(self, uart_values):
        now = time.time()
        if now - self.last_uart_time < 0.5:   # ★ 限制最多 10Hz
            return
        self.last_uart_time = now

        if self.latest_pose is None:
            self.get_logger().info("[融合略過] 尚未取得最新 Pose")
            return
        if len(uart_values) != 6:
            self.get_logger().warn(f"[融合略過] UART 長度非 6: {len(uart_values)}")
            return

        new_vals = [round(p + u, 2) for p, u in zip(self.latest_pose, uart_values)]
        self.get_logger().info(f"[融合結果] {new_vals}")

        if self.controller is not None:
            self.controller.append_tcp(new_vals)
        else:
            self.get_logger().warn("controller 尚未注入，無法送入佇列")

    def destroy_node(self):
        self.uart.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = PoseAndUARTNode()
    tm_controller = TMRobotController()
    tm_controller.setup_services()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(tm_controller)

        node.controller = tm_controller  # 注入 controller
        executor.spin()                 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        tm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
