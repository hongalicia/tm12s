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
            
        script = f'PTP("JPP",{joint_values[0]:.2f}, {joint_values[1]:.2f}, {joint_values[2]:.2f}, ' \
                 f'{joint_values[3]:.2f}, {joint_values[4]:.2f}, {joint_values[5]:.2f},' \
                 f'20,20,50,true)'
        
        self.joint_queue.append(script)
        self.get_logger().info(f"已將腳本加入佇列: {script}")
    
def main():
    rclpy.init()
    node = TMRobotController()
    
    try:
        node.setup_services()
        

        
        node.append_joint([1.04, -0.17, 67.21, -5.53, 97.07, 22.92])


        while node.joint_queue:
            script_to_run = node.joint_queue.popleft()
            node.get_logger().info(f"正在執行佇列中的腳本: {script_to_run}")
            node.send_script(script_to_run)
            time.sleep(1) 
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()