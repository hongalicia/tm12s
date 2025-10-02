#!/usr/bin/env python3
# tcp_echo_cpp_robot2_min.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def _clamp(v, lo, hi): return max(lo, min(hi, v))

def normalize_quat(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (0.0, 0.0, 0.0, 1.0) if n == 0 else (x/n, y/n, z/n, w/n)

def quat_to_euler_zyx_deg(qx, qy, qz, qw):
    qx, qy, qz, qw = normalize_quat(qx, qy, qz, qw)
    # yaw (Z)
    siny = 2.0 * (qw*qz + qx*qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny, cosy)
    # pitch (Y)
    sinp = 2.0 * (qw*qy - qz*qx)
    sinp = _clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)
    # roll (X)
    sinr = 2.0 * (qw*qx + qy*qz)
    cosr = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr, cosr)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)  # rx, ry, rz

def xyzquat_to_cpp(x_m, y_m, z_m, qx, qy, qz, qw):
    x_mm, y_mm, z_mm = x_m*1000.0, y_m*1000.0, z_m*1000.0
    rx, ry, rz = quat_to_euler_zyx_deg(qx, qy, qz, qw)
    return [round(x_mm, 2), round(y_mm, 2), round(z_mm, 2),
            round(rx, 2), round(ry, 2), round(rz, 2)]


def pose_to_cpp(pose):
    # 兼容 geometry_msgs.msg.Pose 與 PoseStamped
    if hasattr(pose, "pose"):
        p = pose.pose.position
        q = pose.pose.orientation
    else:
        p = pose.position
        q = pose.orientation
    return xyzquat_to_cpp(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class EchoRobot2CPP(Node):
    def __init__(self):
        super().__init__('echo_robot2_cpp')
        self.sub = self.create_subscription(PoseStamped, '/tool_pose', self.cb, 10)
        self.get_logger().info(' subscribe /tool_pose')
        self.latest_pose = None

    def cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        """ self.get_logger().info(
            f'RAW  pos(m)=({p.x:.5f},{p.y:.5f},{p.z:.5f}), '
            f'quat=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})'
        ) """
        # 2)  transform to CPP（x y z rx ry rz）
        x_mm, y_mm, z_mm = p.x*1000.0, p.y*1000.0, p.z*1000.0
        rx, ry, rz = quat_to_euler_zyx_deg(q.x, q.y, q.z, q.w)

        list_output = (
            f'[{x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f}]'
        )
        #self.get_logger().info(f'{list_output}')


        arr = pose_to_cpp(msg)   # 呼叫共用轉換函式
        self.latest_pose = arr   # 更新暫存的最新位置
        #self.get_logger().info(f'{arr}')
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

if __name__ == '__main__':
    main()