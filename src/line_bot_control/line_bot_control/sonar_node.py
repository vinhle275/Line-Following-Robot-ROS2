import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')
        
        self.declare_parameter('sonar_filter_window', 10)
        self.window_size = self.get_parameter('sonar_filter_window').value
        self.history = []
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Áp dụng QoS vào Subscription
        self.create_subscription(LaserScan, '/sensors/sonar_left_raw', self.left_cb, qos_profile)
        self.create_subscription(LaserScan, '/sensors/sonar_right_raw', self.right_cb, qos_profile)
        
        self.dist_left = 99.0
        self.dist_right = 99.0
        self.pub_dist = self.create_publisher(Float32, '/sensors/sonar/filtered', 10)
        
        self.create_timer(0.05, self.process)

    def get_val(self, msg):
        if not msg.ranges: return 99.0
        val = min(msg.ranges)
        return val if val > 0.05 else 99.0

    def left_cb(self, msg): self.dist_left = self.get_val(msg)
    def right_cb(self, msg): self.dist_right = self.get_val(msg)

    def process(self):
        current_min = min(self.dist_left, self.dist_right)
        
        self.history.append(current_min)
        if len(self.history) > self.window_size:
            self.history.pop(0)
            
        avg = sum(self.history) / len(self.history)
        
        msg = Float32()
        msg.data = avg
        self.pub_dist.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()