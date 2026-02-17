#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy # Import thêm QoS

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain')
        
        # --- Parameters ---
        self.declare_parameter('line_threshold', 0.028)
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('turn_speed', 0.25)
        self.declare_parameter('safe_distance_m', 0.5)

        self.threshold = self.get_parameter('line_threshold').value
        self.base_speed = self.get_parameter('base_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safe_dist = self.get_parameter('safe_distance_m').value

        # --- CẤU HÌNH QOS ---
        # Cho phép nhận dữ liệu Best Effort (thường dùng cho sensor)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # --- Subscribers với QoS Profile ---
        self.create_subscription(LaserScan, '/sensors/line_left_raw', self.left_cb, qos_profile)
        self.create_subscription(LaserScan, '/sensors/line_right_raw', self.right_cb, qos_profile)
        self.create_subscription(LaserScan, '/sensors/sonar_front_raw', self.sonar_cb, qos_profile)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_debug = self.create_publisher(String, '/robot/debug_state', 10)

        # State Variables
        self.sensor_left = 0
        self.sensor_right = 0
        self.raw_left = 9.9
        self.raw_right = 9.9
        self.sonar_dist = 9.9
        
        self.create_timer(0.05, self.control_loop)

    def process_sensor(self, msg):
        if not msg.ranges: return 9.9, 0
        raw_dist = msg.ranges[0]
        # Logic: < Threshold là Line (1), ngược lại là Đất (0)
        state = 1 if raw_dist < self.threshold else 0
        return raw_dist, state

    def left_cb(self, msg): 
        self.raw_left, self.sensor_left = self.process_sensor(msg)
        
    def right_cb(self, msg): 
        self.raw_right, self.sensor_right = self.process_sensor(msg)
    
    def sonar_cb(self, msg): 
        if msg.ranges:
             valid = [r for r in msg.ranges if r > 0.02]
             if valid: self.sonar_dist = min(valid)

    def control_loop(self):
        msg = Twist()
        debug_msg = ""
        
        # In Log để debug
        self.get_logger().info(
            f"L_raw: {self.raw_left:.4f} | R_raw: {self.raw_right:.4f} | STATE: {self.sensor_left}-{self.sensor_right}"
        )

        # Logic điều khiển
        if self.sonar_dist < self.safe_dist:
            msg.linear.x = 0.0
            debug_msg = "OBSTACLE"
        else:
            if self.sensor_left == 1 and self.sensor_right == 1:
                msg.linear.x = self.base_speed
                msg.angular.z = 0.0
                debug_msg = "FORWARD"
            elif self.sensor_left == 0 and self.sensor_right == 1:
                msg.linear.x = self.base_speed * 0.5
                msg.angular.z = -self.turn_speed 
                debug_msg = "TURN RIGHT"
            elif self.sensor_left == 1 and self.sensor_right == 0:
                msg.linear.x = self.base_speed * 0.5
                msg.angular.z = self.turn_speed
                debug_msg = "TURN LEFT"
            else:
                # Mất line -> Lùi nhẹ
                msg.linear.x = -0.05
                msg.angular.z = 0.0
                debug_msg = "LOST"

        self.pub_cmd.publish(msg)
        self.pub_debug.publish(String(data=f"{debug_msg}|Err:0|Spd:{msg.linear.x:.2f}|L:{self.sensor_left}|R:{self.sensor_right}"))

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()