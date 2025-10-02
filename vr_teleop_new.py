#!/usr/bin/env python3
import socket
import numpy as np
import math
import time
import threading

import rclpy
from transfor_TCP import *
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from one_arm_control_CPP import TMRobotController

from rclpy.logging import LoggingSeverity
from tm_msgs.srv import SendScript


def start_ros_listener():
    rclpy.init()
    node = EchoRobot2CPP()  # 你現成的訂閱者，內部會把 /tool_pose 轉成 CPP(mm,deg)

    # add: 降低這個 node 的輸出層級，避免你不想要的 [INFO] spam
    try:
        node.get_logger().set_level(LoggingSeverity.ERROR)
    except Exception:
        pass

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass  # add: 關閉時常見例外，忽略避免紅字

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    return node, executor, spin_thread


# 修改：加入角度死區（若你暫不送角度，這段也不會誤觸）
def _should_send_tcp(prev_tcp, curr_tcp, last_time, min_dt, mm_deadband, deg_deadband):
    import time
    now = time.time()
    if (now - last_time) < min_dt:
        return False, last_time
    if prev_tcp is None:
        return True, now
    # 位置死區（任一軸超過就送）
    for i in range(3):  # x,y,z in mm
        if abs(curr_tcp[i] - prev_tcp[i]) >= mm_deadband:
            return True, now
    # 新增：角度死區（rx,ry,rz in deg）
    for i in range(3, 6):
        if abs(curr_tcp[i] - prev_tcp[i]) >= deg_deadband:
            return True, now
    return False, last_time


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def apply_rotation_delta(rotation_curr, rotation_prev):
    """
    rotation_*: [qx, qy, qz, qw]
    return: (d_rx, d_ry, d_rz) in deg, 已做 wrap 到 [-180, 180]
    """
    rx, ry, rz = quat_to_euler_zyx_deg(*rotation_curr)
    prev_rx, prev_ry, prev_rz = quat_to_euler_zyx_deg(*rotation_prev)
    d_rx = _wrap_deg(rx - prev_rx)
    d_ry = _wrap_deg(ry - prev_ry)
    d_rz = _wrap_deg(rz - prev_rz)
    return d_rx, d_ry, d_rz


# 新增：五次多項式 blend 函數 alpha(s) 與產生子點工具
def _alpha(s: float) -> float:  # 新增
    # 6s^5 - 15s^4 + 10s^3  (C^2 連續，端點速度/加速度為 0)
    return ((6*s - 15)*s + 10)*s*s*s

def poly_blend_points(last_tcp, target_tcp, steps: int):  # 新增
    if steps <= 1:
        return [target_tcp[:]]
    pts = []
    for k in range(1, steps + 1):
        s = k / steps
        a = _alpha(s)
        out = []
        for last_v, tgt_v in zip(last_tcp, target_tcp):
            out.append(last_v + a * (tgt_v - last_v))
        # 角度可視需要再做 wrap，這裡沿用線性插值值即可
        out[3] = _wrap_deg(out[3])
        out[4] = _wrap_deg(out[4])
        out[5] = _wrap_deg(out[5])
        pts.append(out)
    return pts


def main():
    # start ROS listening and TM controller
    pose_node, executor, spin_thread = start_ros_listener()
    tm_node = TMRobotController()
    tm_node.setup_services()
    executor.add_node(tm_node)

    # 新增：上一筆送出的目標（同時拿來當下一次 base 與 deadband 比較）
    last_cmd_tcp = None  # 新增
    _last_sent_time = 0.0
    # 修改：調快一點，降低停頓感（可微調 0.15~0.30）
    min_cmd_interval_s = 0.20
    min_mm_delta = 1.0
    min_deg_delta = 2.0

    # 新增：blending 參數
    blend_T = 0.40      # 新增：過渡視窗時間（秒）
    blend_rate = 40     # 新增：插值頻率（Hz）
    max_steps = 120     # 新增：最多產生的子點（避免爆量）
    # 實際 steps
    steps_default = max(1, min(max_steps, int(round(blend_T * blend_rate))))

    host = "0.0.0.0"
    port = 5050

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.bind((host, port))
    client_socket.listen()
    print(f"Listening on port {port}")
    conn, addr = client_socket.accept()
    print(f"Connection from {addr}")

    first_capture = True
    acc_x_mm = acc_y_mm = acc_z_mm = 0.0
    acc_rx = acc_ry = acc_rz = 0.0

    try:
        while True:
            data = conn.recv(1024).decode("utf-8").strip()
            if not data:
                break
            lines = data.split('\n')
            if not lines or not lines[0]:
                continue

            # 修改：Latest-wins，用最後一行
            data = lines[-1].split(',')
            if len(data) < 9:
                continue

            print("Received data:", data)

            values = list(map(float, data))
            positionR = values[:3]
            rotationR = values[3:7]
            buttonR = values[7]
            print("buttonR: ", buttonR)

            positionL = values[8:11] if len(values) >= 11 else [0.0, 0.0, 0.0]
            rotationL = values[11:15] if len(values) >= 15 else [0.0, 0.0, 0.0, 1.0]
            buttonL = values[15] if len(values) >= 16 else 0.0
            print("buttonL: ", buttonL)

            if first_capture:
                prev_positionR = positionR
                prev_rotationR = rotationR
                prev_positionL = positionL
                prev_rotationL = rotationL
                first_capture = False
                continue  # add: 第一幀只當基準，不送

            shift_positionR = [a - b for a, b in zip(positionR, prev_positionR)]
            shift_positionL = [a - b for a, b in zip(positionL, prev_positionL)]
            print("shift_positionR: ", shift_positionR)
            print("shift_positionL: ", shift_positionL)

            # ros get current end-position
            if not pose_node.latest_pose:
                print("[ROS] Current end-position: <waiting for /tool_pose...>")
                # 避免解包 None，直接跳過這幀
                prev_positionR = positionR
                prev_rotationR = rotationR
                prev_positionL = positionL
                prev_rotationL = rotationL
                continue
            else:
                print("[ROS] Current end-position:", pose_node.latest_pose)

            # 用上一筆送出的目標當 base（第一次才用量測）
            if last_cmd_tcp is None:
                base_x, base_y, base_z, base_rx, base_ry, base_rz = pose_node.latest_pose
            else:
                base_x, base_y, base_z, base_rx, base_ry, base_rz = last_cmd_tcp

            d_rx, d_ry, d_rz = apply_rotation_delta(rotationR, prev_rotationR)

            # 累加進暫存器（送出後清零）
            acc_x_mm += shift_positionR[0] * 1000.0
            acc_y_mm += shift_positionR[1] * 1000.0
            acc_z_mm += shift_positionR[2] * 1000.0
            # 若你現在不送角度，先保留累加但不套用；要用時開啟下面三行
            # acc_rx   = _wrap_deg(acc_rx + d_rx)
            # acc_ry   = _wrap_deg(acc_ry + d_ry)
            # acc_rz   = _wrap_deg(acc_rz + d_rz)

            # 目標: base + 累加位置；角度沿用 base（之後要 blend 角度再開）
            new_x = base_x + acc_x_mm
            new_y = base_y + acc_y_mm
            new_z = base_z + acc_z_mm
            new_rx, new_ry, new_rz = base_rx, base_ry, base_rz
            target_tcp = [new_x, new_y, new_z, new_rx, new_ry, new_rz]

            # Debug：看看累積與目標
            print("[ROS] acc:", [round(acc_x_mm,2), round(acc_y_mm,2), round(acc_z_mm,2)])
            print("[ROS] Target CPP:", target_tcp)

            # 判斷是否該送（使用上一筆送出的點做 deadband 比較）
            should_send, _last_sent_time = _should_send_tcp(
                last_cmd_tcp, target_tcp, _last_sent_time,
                min_cmd_interval_s, min_mm_delta, min_deg_delta
            )

            if should_send:
                # 建議：不要每次都清；避免中斷規劃。只有太長才清。
                if hasattr(tm_node, "tcp_queue"):
                    try:
                        if len(tm_node.tcp_queue) > 5:
                            tm_node.tcp_queue.clear()
                    except Exception:
                        pass

                # 新增：五次多項式時間窗 blending（位置三軸 + 目前角度）
                # 上一筆若不存在，直接當作已在 last；否則以 last_cmd_tcp 為起點
                if last_cmd_tcp is None:
                    last_for_blend = pose_node.latest_pose[:]  # 保底
                else:
                    last_for_blend = last_cmd_tcp[:]

                steps = steps_default  # 你也可以依距離調整 steps（距離越大步數越多）
                blend_points = poly_blend_points(last_for_blend, target_tcp, steps)

                # 批次入列：控制器會按序執行，動作連續不頓
                for p in blend_points:
                    tm_node.append_tcp(p)

                # 更新上一筆送出的點
                last_cmd_tcp = target_tcp[:]

                # 送出即清零，開始累下一批 Δ
                acc_x_mm = acc_y_mm = acc_z_mm = 0.0
                acc_rx = acc_ry = acc_rz = 0.0

            # 更新上一幀基準
            prev_positionR = positionR
            prev_rotationR = rotationR
            prev_positionL = positionL
            prev_rotationL = rotationL

            print("========================================================================")

    except KeyboardInterrupt:
        print("Closing connection...")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        client_socket.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
