# main.py
import rclpy
from transfor_TCP import EchoRobot2CPP

def main():
    rclpy.init()
    node = EchoRobot2CPP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
