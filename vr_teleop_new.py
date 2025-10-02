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


def _should_send_tcp(prev_tcp, curr_tcp, last_time, min_dt, mm_deadband, deg_deadband):  # add
    import time
    now = time.time()
    if (now - last_time) < min_dt:
        return False, last_time
    if prev_tcp is None:
        return True, now
    # 位置/角度變化檢查（任一軸超過死區就送）
    for i in range(3):  # x,y,z in mm
        if abs(curr_tcp[i] - prev_tcp[i]) >= mm_deadband:
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



def main():
    # start ROS listening and TM controller
    pose_node, executor, spin_thread = start_ros_listener()
    tm_node = TMRobotController()
    tm_node.setup_services()
    executor.add_node(tm_node)
    last_cmd_tcp = None
    _last_sent_time = 0.0 
    min_cmd_interval_s = 1.0       # add: 最小送指令間隔（10Hz 上限）
    min_mm_delta = 2.0              # add: 位置死區（mm），任一軸超過才送
    min_deg_delta = 2.0             # add: 姿態死區（deg），任一軸超過才送
   

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
            # Split the data by newline
            data = data.split('\n')
            if not data or not data[0]:
                continue
            
            # Parse data: right hand-x, y, z, qx, qy, qz, qw, button, and left hand-x, y, z, qx, qy, qz, qw, button.
            # Use the latest one
            #data = data[0].split(',')
            data = data[-1].split(',')
            # partial data, reject
            if len(data) < 9:
                continue

            print("Received data:", data)

            values = list(map(float, data))
            positionR = values[:3]
            #print("positionR: ", positionR)
            rotationR = values[3:7]
            #print("rotationR: ", rotationR)
            buttonR = values[7]
            print("buttonR: ", buttonR)

            positionL = values[8:11]
            #print("positionL: ", positionL)
            rotationL = values[11:15]
            #print("rotationL: ", rotationL)
            buttonL = values[15]
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
                prev_positionR = positionR
                prev_rotationR = rotationR
                continue
            else:
                print("[ROS] Current end-position:", pose_node.latest_pose)

            if last_cmd_tcp is None:  
                base_x, base_y, base_z, base_rx, base_ry, base_rz = pose_node.latest_pose  
            else:  
                base_x, base_y, base_z, base_rx, base_ry, base_rz = last_cmd_tcp  
           

            d_rx, d_ry, d_rz = apply_rotation_delta(rotationR, prev_rotationR)
            dx_mm = shift_positionR[0] * 1000.0
            dy_mm = shift_positionR[1] * 1000.0
            dz_mm = shift_positionR[2] * 1000.0
            #new_x = base_x + dx_mm
            #new_y = base_y + dy_mm
            #new_z = base_z + dz_mm
            new_rx, new_ry, new_rz = base_rx, base_ry, base_rz
            #new_rx = _wrap_deg(base_rx + d_rx)
            #new_ry = _wrap_deg(base_ry + d_ry)
            #new_rz = _wrap_deg(base_rz + d_rz)

            # 累加進暫存器
            acc_x_mm += shift_positionR[0] * 1000.0
            acc_y_mm += shift_positionR[1] * 1000.0
            acc_z_mm += shift_positionR[2] * 1000.0
            acc_rx   = _wrap_deg(acc_rx + d_rx)
            acc_ry   = _wrap_deg(acc_ry + d_ry)
            acc_rz   = _wrap_deg(acc_rz + d_rz)
            print("[ROS] acc:", [
                round(acc_x_mm, 2),
                round(acc_y_mm, 2),
                round(acc_z_mm, 2),
                round(acc_rx, 2),
                round(acc_ry, 2),
                round(acc_rz, 2)
            ])
            new_x = base_x + acc_x_mm
            new_y = base_y + acc_y_mm
            new_z = base_z + acc_z_mm
            #new_rx = _wrap_deg(base_rx + acc_rx)
            #new_ry = _wrap_deg(base_ry + acc_ry)
            #new_rz = _wrap_deg(base_rz + acc_rz)
            # ros set new end-position
            target_tcp = [new_x, new_y, new_z, new_rx, new_ry, new_rz]
            print("[ROS] Target CPP:", target_tcp)
            now = time.time()
            fake_val = [674.57, -155.37, 697.42, 175.11, -0.7, 90.13]
            if hasattr(tm_node, "tcp_queue"):
                try:
                    tm_node.tcp_queue.clear()
                except Exception:
                    pass
            should_send, _last_sent_time = _should_send_tcp(
            last_cmd_tcp, target_tcp, _last_sent_time,
            min_cmd_interval_s, min_mm_delta, min_deg_delta
            )
            if should_send:
                tm_node.append_tcp(target_tcp)
                last_cmd_tcp = target_tcp[:]
                acc_x_mm = acc_y_mm = acc_z_mm = 0.0
                acc_rx = acc_ry = acc_rz = 0.0
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
