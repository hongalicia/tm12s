# one_arm_control_CPP.py
import time
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from collections import deque

class TMRobotController(Node):
    def __init__(self):
        super().__init__('tm_robot_controller')
        self.script_cli = None
        self.tcp_queue = deque()
        self._busy = False
        self._min_send_interval = 0.20  # 秒
        self._last_send_ts = 0.0

        self.no_retry_on_fail = True  # add
        self.clear_queue_on_fail = False  # add

        # ★ 每 50ms 處理一次佇列
        self.create_timer(0.05, self._process_queue)

    def setup_services(self):
        self.get_logger().info("等待 ROS 2 服務啟動...")
        self.script_cli = self.create_client(SendScript, 'send_script')
        while not self.script_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 send_script 服務...')

    def append_tcp(self, tcp_values: list, vel=80, acc=200, coord=80, fine=False):
        if len(tcp_values) != 6:
            self.get_logger().error("TCP 必須 6 個數字")
            return
        fine_str = "true" if fine else "false"
        # ★ 改用 TCP（TM Script 標準）
        script = (f'PTP("CPP",{tcp_values[0]:.2f}, {tcp_values[1]:.2f}, {tcp_values[2]:.2f}, '
                  f'{tcp_values[3]:.2f}, {tcp_values[4]:.2f}, {tcp_values[5]:.2f},'
                  f'{vel},{acc},{coord},{fine_str})')
        self.tcp_queue.append(script)
        self.get_logger().info(f"已將腳本加入佇列: {script}")

    def _process_queue(self):
        if self._busy or not self.tcp_queue:
            return
        now = time.time()
        if now - self._last_send_ts < self._min_send_interval:
            return

        script_to_run = self.tcp_queue.popleft()
        self.get_logger().info(f"正在執行佇列中的腳本: {script_to_run}")
        self._send_script_async(script_to_run)
        self._last_send_ts = now
        self._busy = True

    def _send_script_async(self, script: str):
        if not self.script_cli:
            self.get_logger().error("send_script 客戶端尚未初始化。")
            self._busy = False
            return
        req = SendScript.Request()
        req.id = "auto"
        req.script = script
        future = self.script_cli.call_async(req)

        def _done(_):
            try:
                res = future.result()
                ok = bool(getattr(res, "ok", False))
                if ok:
                    self.get_logger().info("✅ 執行成功")
                else:
                    self.get_logger().warn("⚠️ 執行失敗：跳過該指令，不重送")
                    if self.clear_queue_on_fail:  # add
                        try:
                            qn = len(self.tcp_queue)
                            self.tcp_queue.clear()
                            self.get_logger().warn(f"⚠️ 已清空剩餘佇列（{qn} 筆）")  # add
                        except Exception as e:
                            self.get_logger().error(f"清空佇列失敗：{e}")  # add

                    # remove: 若要舊行為（失敗回推等待重送），取消註解下面 2 行
                    # if not self.no_retry_on_fail:
                    #     self.tcp_queue.appendleft(script)

            except Exception as e:
                self.get_logger().error(f"[SendScript 失敗] {e}（已跳過，不重送）")
                if self.clear_queue_on_fail:  # add
                    try:
                        qn = len(self.tcp_queue)
                        self.tcp_queue.clear()
                        self.get_logger().warn(f"⚠️ 已清空剩餘佇列（{qn} 筆）")  # add
                    except Exception as ee:
                        self.get_logger().error(f"清空佇列失敗：{ee}")  # add
            finally:
                self._busy = False

        future.add_done_callback(_done)

    # add: 提供外部（例如 VR 控制程式）在送新命令前主動清空殘留佇列
    def clear_queue(self):
        try:
            n = len(self.tcp_queue)
            self.tcp_queue.clear()
            self.get_logger().info(f"已清空佇列，共 {n} 筆")
        except Exception as e:
            self.get_logger().error(f"清空佇列失敗：{e}")
