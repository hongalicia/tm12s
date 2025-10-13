#!/usr/bin/env python3
# pose_uart_node.py
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from transfor_TCP import *         
from one_arm_control_CPP import TMRobotController
from uart import UART


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0

def _lerp_angle_deg(a: float, b: float, t: float) -> float:
    delta = ((b - a + 180.0) % 360.0) - 180.0
    return _wrap_deg(a + delta * t)

def _alpha(s: float) -> float:
    return ((6*s - 15)*s + 10)*s*s*s

def poly_blend_points(last_tcp, target_tcp, steps: int):
    if steps <= 1:
        return [target_tcp[:]]
    pts = []
    for k in range(1, steps + 1):
        s = k / steps
        a = _alpha(s)
        out = []
        out.append(last_tcp[0] + a * (target_tcp[0] - last_tcp[0]))
        out.append(last_tcp[1] + a * (target_tcp[1] - last_tcp[1]))
        out.append(last_tcp[2] + a * (target_tcp[2] - last_tcp[2]))
        out.append(_lerp_angle_deg(last_tcp[3], target_tcp[3], a))
        out.append(_lerp_angle_deg(last_tcp[4], target_tcp[4], a))
        out.append(_lerp_angle_deg(last_tcp[5], target_tcp[5], a))
        pts.append(out)
    return pts

def _should_send_tcp(prev_tcp, curr_tcp, last_time, min_dt, mm_deadband, deg_deadband):
    now = time.time()
    if (now - last_time) < min_dt:
        return False, last_time
    if prev_tcp is None:
        return True, now
    # 位置死區
    for i in range(3):
        if abs(curr_tcp[i] - prev_tcp[i]) >= mm_deadband:
            return True, now
    # 姿態死區（角度做 wrap）
    for i in range(3, 6):
        if abs(_wrap_deg(curr_tcp[i] - prev_tcp[i])) >= deg_deadband:
            return True, now
    return False, last_time


# ---------- 主節點：Pose + UART ----------
class PoseAndUARTNode(Node):
    def __init__(self, pose_node: Node, tm_node: TMRobotController,
                 uart_port="/dev/ttyUSB0", uart_baud=115200):
        super().__init__("pose_and_uart_node")
        self.pose_node = pose_node
        self.tm_node = tm_node

  
        self.min_cmd_interval_s = 0.20     
        self.min_mm_delta = 5.0            
        self.min_deg_delta = 2.0           
        self.blend_T = 0.40                
        self.blend_rate = 40               
        self.max_steps = 120               
        self.steps_default = max(1, min(self.max_steps, int(round(self.blend_T * self.blend_rate))))

        self._first_uart = True
        self._prev_uart = None             # 上一筆 UART 值 [x,y,z,rx,ry,rz]
        self._last_cmd_tcp = None          # 最近送出的點 (mm,deg)
        self._last_sent_time = 0.0

        # 累積器（送出後清零）
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.acc_rx = 0.0
        self.acc_ry = 0.0
        self.acc_rz = 0.0
        self.count = 0
        # 佇列保護：保留最近 64 筆，避免整批清空
        self.max_queue_cached = 64

        # 互斥避免回呼重入
        self._lock = threading.Lock()

        # 啟動 UART
        self.uart = UART(port=uart_port, baudrate=uart_baud, on_data=self.on_uart)
        self.get_logger().info(f"UART started on {uart_port} @ {uart_baud}")

    # ---- UART 回呼 ----
    def on_uart(self, values):
        # values = [x, y, z, rx, ry, rz]，單位假設已是 mm/deg
        with self._lock:
            if not values or len(values) < 6:
                return

            # 等到 /tool_pose 有資料
            latest_pose = getattr(self.pose_node, "latest_pose", None)
            if latest_pose is None:
                if self._first_uart:
                    self._prev_uart = values[:6]
                    self._first_uart = False
                return

            # 第一次僅建立基準
            if self._first_uart:
                self._prev_uart = values[:6]
                self._first_uart = False
                return


            drx = _wrap_deg(values[3] - self._prev_uart[3])
            dry = _wrap_deg(values[4] - self._prev_uart[4])
            drz = _wrap_deg(values[5] - self._prev_uart[5])

            self.acc_x  += values[0]
            self.acc_y  += values[1]
            self.acc_z  += values[2]
            self.acc_rx  = _wrap_deg(self.acc_rx + drx)
            self.acc_ry  = _wrap_deg(self.acc_ry + dry)
            self.acc_rz  = _wrap_deg(self.acc_rz + drz)

            if self._last_cmd_tcp is None:
                base = latest_pose[:]          
            else:
                base = self._last_cmd_tcp[:]

            target_tcp = [
                base[0] + self.acc_x * 30,
                base[1] ,
                base[2] ,
                _wrap_deg(base[3] ),
                _wrap_deg(base[4] ),
                _wrap_deg(base[5] ),
            ]

            # 4) 節流 + 死區
            should_send, self._last_sent_time = _should_send_tcp(
                self._last_cmd_tcp, target_tcp, self._last_sent_time,
                self.min_cmd_interval_s, self.min_mm_delta, self.min_deg_delta
            )
            self.count += 1
            if should_send:
                # queue 長度保護：丟舊保新
                if hasattr(self.tm_node, "tcp_queue"):
                    try:
                        while len(self.tm_node.tcp_queue) > self.max_queue_cached:
                            self.tm_node.tcp_queue.clear()
                    except Exception:
                        pass

                last_for_blend = latest_pose[:] if (self._last_cmd_tcp is None) else self._last_cmd_tcp[:]
                steps = self.steps_default
                blend_points = poly_blend_points(last_for_blend, target_tcp, steps)

                # 除錯觀測：一次塞幾點、目標是什麼
                self.get_logger().info(
                    f"[BLEND] enqueue {len(blend_points)} points -> target={list(map(lambda x: round(x,2), target_tcp))}"
                )

                #for p in blend_points:
                self.tm_node.append_tcp(target_tcp)
                print(self.count)
                self._last_cmd_tcp = target_tcp[:]
                # 清空累積器
                self.acc_x = self.acc_y = self.acc_z = 0.0
                self.acc_rx = self.acc_ry = self.acc_rz = 0.0
                self.count = 0
            # 5) 更新上一筆 UART
            self._prev_uart = values[:6]


# ---------- 啟動 ----------
def main():
    rclpy.init()

    # /tool_pose 訂閱者（輸出 (mm,deg)）
    pose_node = EchoRobot2CPP()
    try:
        pose_node.get_logger().set_level(LoggingSeverity.ERROR)
    except Exception:
        pass

    # TM 控制器
    tm_node = TMRobotController()
    tm_node.setup_services()

    # Pose + UART 節點
    node = PoseAndUARTNode(pose_node, tm_node, uart_port="/dev/ttyUSB0", uart_baud=115200)

    # Executor
    executor = SingleThreadedExecutor()
    for n in (pose_node, tm_node, node):
        executor.add_node(n)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.uart.close()
        except Exception:
            pass
        for n in (node, tm_node, pose_node):
            try:
                n.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
