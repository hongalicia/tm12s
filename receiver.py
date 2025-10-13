import socket
import threading
import time
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from collections import deque

class TMRobotController(Node):
    def __init__(self):
        super().__init__('tm_robot_controller')
        self.script_cli = None
        self.joint_queue = deque()
        self.received_data_queue = deque()  # 已改為存「完整一行」的 bytes
        self.is_connected = False

        # --- 新增：TCP 行緩衝 ---
        self._rx_buf = b''

        # --- 新增：節流/去抖參數 ---
        self.min_cmd_interval_s = 0.15   # 兩次送指令的最短間隔（避免噴太快）
        self.min_deg_delta = 0.5         # 每關節最小角度變化（小於此視為無效更新）
        self._last_sent_time = 0.0
        self._last_joints = None         # 記錄上一次送出的關節組

        self.setup_services()
        self.start_socket_server()

        # 每 100ms 嘗試處理一行資料（與你原本相同）
        self.create_timer(0.1, self.process_joint_data)

    # ---------------- ROS 2 服務 ----------------
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
        self.script_cli.call_async(req)
        self.get_logger().info(f"已發送腳本: {script}")

    # ---------------- 指令封裝 ----------------
    def append_joint(self, joint_values: list):
        if len(joint_values) != 6:
            self.get_logger().error("關節值列表必須包含 6 個值。")
            return

        # 這裡維持你原本的 TM 指令格式（JPP/度數）。速度/加速度/blend/fine 沿用原本
        script = (
            f'PTP("JPP",{joint_values[0]:.2f}, {joint_values[1]:.2f}, {joint_values[2]:.2f}, '
            f'{joint_values[3]:.2f}, {joint_values[4]:.2f}, {joint_values[5]:.2f},'
            f'20,20,80,true)'
        )
        self.joint_queue.append(script)
        self.get_logger().info(f"add to queue: {script}")

    # ---------------- 節流與去抖 ----------------
    def _should_send(self, joints):
        now = time.time()
        # 間隔太短 → 不送
        if (now - self._last_sent_time) < self.min_cmd_interval_s:
            return False
        # 無上一筆 → 送
        if self._last_joints is None:
            return True
        # 角度變化檢查（任一關節超過 deadband 才送）
        for a, b in zip(self._last_joints, joints):
            if abs(a - b) >= self.min_deg_delta:
                return True
        return False

    def _mark_sent(self, joints):
        self._last_sent_time = time.time()
        self._last_joints = joints

    # ---------------- CSV 行處理 ----------------
    def process_joint_data(self):
        
        if not self.received_data_queue:
            return

        raw_line = self.received_data_queue.popleft()
        try:
            line = raw_line.decode('utf-8', errors='ignore').strip()
           
            parts = line.split(',')
            if len(parts) < 7:
                self.get_logger().error(f"資料欄位不足（收到 {len(parts)} 項）: {line}")
                return

            # 轉 float：忽略 parts[0] (timestamp)，只取 6 個關節
            # 注意：若資料有 +/- 符號或小數皆可正確處理
            joint_values = [float(p) for p in parts[1:7]]

            # 節流 / 去抖：若不該送，直接略過即可
            if not self._should_send(joint_values):
                return

            # 入列 + 立即送出一筆（維持你原本的流程與節奏，但更安全）
            self.append_joint(joint_values)
            if self.joint_queue:
                script_to_run = self.joint_queue.popleft()
                self.send_script(script_to_run)
                self._mark_sent(joint_values)

        except ValueError:
            self.get_logger().error(f"無法將資料轉換為數字: {raw_line!r}")
        except Exception as e:
            self.get_logger().error(f"處理資料時發生錯誤: {e}")

    # ---------------- Socket 伺服器 ----------------
    def start_socket_server(self):
        host = "0.0.0.0"
        port = 5050

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 避免 TIME_WAIT 綁死
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen()

        self.get_logger().info(f"監聽中 port {port}")

        threading.Thread(target=self.accept_connections, daemon=True).start()

    def accept_connections(self):
        try:
            conn, addr = self.server_socket.accept()
            self.is_connected = True
            self.get_logger().info(f"連線來自 {addr}")

            with conn:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    self._rx_buf += chunk
                    *lines, remain = self._rx_buf.split(b'\n')
                    self._rx_buf = remain 
                    for ln in lines:
                        if ln.strip():
                            self.received_data_queue.append(ln)
        except Exception as e:
            self.get_logger().error(f"Socket 伺服器發生錯誤: {e}")
        finally:
            self.is_connected = False
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.get_logger().info("Socket 伺服器已關閉。")

    def stop_socket_server(self):
        self.is_connected = False
        try:
            self.server_socket.close()
        except Exception:
            pass

def main():
    rclpy.init()
    node = TMRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_socket_server()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
