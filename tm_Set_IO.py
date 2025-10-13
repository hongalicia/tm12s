import subprocess
import time
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript, SetIO

class TMRobotController(Node):
    def __init__(self, robot_ip: str):
        super().__init__('tm_robot_controller')
        self.robot_ip = robot_ip
        self.driver_process = None
        self.script_cli = None
        self.io_cli = None
        
    def start_tm_driver(self):
        self.get_logger().info(f"啟動 tm_driver，IP: {self.robot_ip}")
        self.driver_process = subprocess.Popen([
            "ros2", "launch", "tm_driver", "tm_bringup.launch.py",
            f"robot_ip:={self.robot_ip}"
        ])
        time.sleep(5)
        
        self.script_cli = self.create_client(SendScript, 'send_script')
        while not self.script_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 send_script 服務啟動...')
        
        self.io_cli = self.create_client(SetIO, 'set_io')
        while not self.io_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_io 服務啟動...')
    
    def send_script(self, script: str):
        req = SendScript.Request()
        req.id = "auto"
        req.script = script
        future = self.script_cli.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                msg = "✅ 執行成功" if future.result().ok else "⚠️ 執行失敗"
                self.get_logger().info(msg)
                break
    
    
    def stop_tm_driver(self):
        if self.driver_process:
            self.driver_process.terminate()
            self.driver_process.wait()
            self.get_logger().info("tm_driver 已關閉")
    
 

def main():
    rclpy.init()
    node = TMRobotController("192.168.1.10")
    
    try:
        node.start_tm_driver()
        
        for s in [
            
            'PTP("JPP",8.16, -7.79, 75.63, 25.77, 89.51, -2.05,35,200,0,true)',
            

        ]:
            node.send_script(s)
        
        
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_tm_driver()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
