#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import csv
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from collections import deque

CSV_PATH = "/home/j300/tm2_ws/positions_log.csv"  # 讀檔路徑（度）

class TMRobotController(Node):
    def __init__(self):
        super().__init__('tm_robot_controller')
        self.script_cli = None
        self.joint_queue = deque()
        
    def setup_services(self):
        self.get_logger().info("等待 ROS 2 服務啟動...")
        self.script_cli = self.create_client(SendScript, 'send_script')
        while not self.script_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 send_script 服務...')
    
    def send_script(self, script: str):
        if not self.script_cli:
            self.get_logger().error("send_script 客戶端尚未初始化。")
            return
            
        req = SendScript.Request()
        req.id = "auto"
        req.script = script
        future = self.script_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            if future.result().ok:
                self.get_logger().info("✅ 執行成功")
            else:
                self.get_logger().info("⚠️ 執行失敗")
    
    def append_joint(self, joint_values: list):
        if len(joint_values) != 6:
            self.get_logger().error("關節值列表必須包含 6 個值。")
            return
            
        script = (
            f'PTP("JPP",{joint_values[0]:.2f}, {joint_values[1]:.2f}, {joint_values[2]:.2f}, '
            f'{joint_values[3]:.2f}, {joint_values[4]:.2f}, {joint_values[5]:.2f},'
            f'20,20,50,true)'
        )
        self.joint_queue.append(script)
        self.get_logger().info(f"已將腳本加入佇列: {script}")

def read_csv_and_queue(node: TMRobotController, csv_path: str):
    try:
        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            first = next(reader, None)
            if first is None:
                node.get_logger().warn("CSV 是空的。")
                return
            def _is_number(s):
                try:
                    float(s); return True
                except: return False

            # 判斷第一列是否標頭
            is_header = not all(_is_number(x) for x in first[:6]) and not (_is_number(first[0]) and _is_number(first[1]))
            if not is_header:
                # 第一列就是數據，先處理它
                row = first
                vals = []
                if len(row) >= 7 and _is_number(row[1]):
                    vals = [float(row[i]) for i in range(1, 7)]  # 跳過 timestamp
                else:
                    vals = [float(row[i]) for i in range(0, 6)]
                node.get_logger().info(f"讀到第 1 列 joints(度)：{vals}")
                node.append_joint(vals)

            # 其餘各列
            for idx, row in enumerate(reader, start=(2 if not is_header else 1)):
                if not row:
                    continue
                try:
                    if len(row) >= 7 and _is_number(row[1]):  # 有 timestamp
                        vals = [float(row[i]) for i in range(1, 7)]
                    else:
                        vals = [float(row[i]) for i in range(0, 6)]
                except Exception:
                    node.get_logger().warn(f"第 {idx} 列解析失敗，已略過：{row}")
                    continue
                node.get_logger().info(f"讀到第 {idx} 列 joints(度)：{vals}")
                node.append_joint(vals)

    except FileNotFoundError:
        node.get_logger().error(f"找不到 CSV 檔案：{csv_path}")
    except Exception as e:
        node.get_logger().error(f"讀取 CSV 發生錯誤：{e}")

def main():
    rclpy.init()
    node = TMRobotController()
    
    try:
        node.setup_services()

        # === 新增：讀檔並把每列加入佇列 ===
        read_csv_and_queue(node, CSV_PATH)

        # 依序送出佇列中的腳本
        while node.joint_queue:
            script_to_run = node.joint_queue.popleft()
            node.get_logger().info(f"正在執行佇列中的腳本: {script_to_run}")
            node.send_script(script_to_run)
            time.sleep(1)  # 簡單間隔，避免過快送指令
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
