#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class LineNode(Node):
    def __init__(self):
        super().__init__('line_node')
        
        # --- CẬP NHẬT MỚI: Đọc tham số threshold từ file yaml ---
        self.declare_parameter('line_threshold', 0.0275)
        self.threshold = self.get_parameter('line_threshold').value
        
        self.create_subscription(LaserScan, '/sensors/line_left_raw', self.left_cb, 10)
        self.create_subscription(LaserScan, '/sensors/line_right_raw', self.right_cb, 10)
        
        self.pub_status = self.create_publisher(String, '/sensors/line/status', 10)
        self.left_detected = False
        self.right_detected = False
        
        # Tần số update cao (50Hz) để bắt kịp tốc độ vật lý
        self.create_timer(0.02, self.update_logic)

    def check_line(self, msg):
        if not msg.ranges: return False
        
        # Cập nhật threshold động (nếu tham số thay đổi runtime)
        # Nếu muốn tối ưu tốc độ, có thể bỏ dòng này và chỉ lấy self.threshold
        current_threshold = self.get_parameter('line_threshold').value
        
        # Logic: Khoảng cách < Threshold nghĩa là đang gặp vật cản (Line nổi)
        if msg.ranges[0] < current_threshold: 
            return True
        return False

    def left_cb(self, msg): self.left_detected = self.check_line(msg)
    def right_cb(self, msg): self.right_detected = self.check_line(msg)

    def update_logic(self):
        state = "CENTER"
        if self.left_detected and self.right_detected:
            state = "CROSSING"
        elif self.left_detected:
            state = "LEFT"
        elif self.right_detected:
            state = "RIGHT"
        else:
            state = "CENTER"
        
        msg = String()
        msg.data = state
        self.pub_status.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()