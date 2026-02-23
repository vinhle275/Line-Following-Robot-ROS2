#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LineNode(Node):
    def __init__(self):
        super().__init__('line_node')
        
        # Khai báo parameter line_threshold
        self.declare_parameter('line_threshold', 0.0295) 
        self.threshold = self.get_parameter('line_threshold').value

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subcribe dữ liệu thô
        self.create_subscription(LaserScan, '/sensors/line_left_raw', self.left_raw_cb, qos_profile)
        self.create_subscription(LaserScan, '/sensors/line_right_raw', self.right_raw_cb, qos_profile)
        
        # Publish dữ liệu đã xử lý (0 hoặc 1)
        self.pub_left = self.create_publisher(Int8, '/sensors/line_left', 10)
        self.pub_right = self.create_publisher(Int8, '/sensors/line_right', 10)

        self.get_logger().info('Line Node has been started.')

    def process_sensor(self, msg):
        if not msg.ranges: 
            return 0
        return 1 if msg.ranges[0] < self.threshold else 0

    def left_raw_cb(self, msg):
        val = self.process_sensor(msg)
        out_msg = Int8()
        out_msg.data = val
        self.pub_left.publish(out_msg)

    def right_raw_cb(self, msg):
        val = self.process_sensor(msg)
        out_msg = Int8()
        out_msg.data = val
        self.pub_right.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()