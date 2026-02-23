import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int8

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain')
        
        # --- Parameters ---
        self.declare_parameter('base_speed', 0.7)        
        self.declare_parameter('turn_speed', 1.2)        
        self.declare_parameter('safe_distance_m', 0.5)

        # Hệ số alpha cho bộ lọc làm mượt EMA (0 < alpha <= 1).
        self.declare_parameter('alpha_v', 0.15)
        self.declare_parameter('alpha_w', 0.25)

        self.base_speed = self.get_parameter('base_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safe_dist = self.get_parameter('safe_distance_m').value

        self.alpha_v = self.get_parameter('alpha_v').value
        self.alpha_w = self.get_parameter('alpha_w').value

        # Hệ số bộ điều khiển PD
        self.Kp = self.turn_speed
        self.Kd = self.turn_speed * 0.35

        self.current_v = 0.0
        self.current_w = 0.0

        # Subscribe topic
        self.create_subscription(Int8, '/sensors/line_left', self.left_cb, 10)
        self.create_subscription(Int8, '/sensors/line_right', self.right_cb, 10)
        self.create_subscription(Float32, '/sensors/sonar/filtered', self.sonar_cb, 10)
        
        # Khởi tạo Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_debug = self.create_publisher(String, '/robot/debug_state', 10)

        self.sensor_left = 0
        self.sensor_right = 0
        self.sonar_dist = 9.9
        
        # Các biến phục vụ thuật toán điều khiển
        self.last_error = 0.0       
        self.last_turn_dir = 0.0    
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Robot Brain Node has been started.')

    # Hàm cập nhật trạng thái

    def left_cb(self, msg): self.sensor_left = msg.data
    def right_cb(self, msg): self.sensor_right = msg.data
    def sonar_cb(self, msg): self.sonar_dist = msg.data

    def control_loop(self):
        msg = Twist()
        debug_msg = ""
        current_error = 0.0
        target_v = 0.0
        target_w = 0.0
        
        # 1. TRÁNH VẬT CẢN
        if self.sonar_dist < self.safe_dist:
            self.current_v = 0.0
            self.current_w = 0.0
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub_cmd.publish(msg)
            self.pub_debug.publish(String(data=f"OBSTACLE STOP|Err:0|Spd:0.0|L:{self.sensor_left}|R:{self.sensor_right}"))
            return

        # 2. TÍNH TOÁN SAI SỐ VÀ CẬP NHẬT TRÍ NHỚ
        if self.sensor_left == 1 and self.sensor_right == 0:
            current_error = 1.0   
            self.last_turn_dir = 1.0 
            debug_msg = "CORRECT LEFT"
            
        elif self.sensor_left == 0 and self.sensor_right == 1:
            current_error = -1.0  
            self.last_turn_dir = -1.0 
            debug_msg = "CORRECT RIGHT"
            
        elif self.sensor_left == 0 and self.sensor_right == 0:
            if self.last_turn_dir > 0:
                current_error = 1.5
                debug_msg = "SEARCHING LEFT (90 DEG)"
            elif self.last_turn_dir < 0:
                current_error = -1.5
                debug_msg = "SEARCHING RIGHT (90 DEG)"
            else:
                current_error = 0.0
                debug_msg = "FORWARD"
                
        else:
            current_error = 0.0
            self.last_turn_dir = 0.0  
            debug_msg = "CENTERED"

        # 3. THUẬT TOÁN PD
        P_term = self.Kp * current_error
        D_term = self.Kd * (current_error - self.last_error)
        target_w = P_term + D_term
        
        self.last_error = current_error 
        
        # 4. CHIẾN LƯỢC VẬN TỐC
        if abs(current_error) >= 1.5:
            target_v = self.base_speed * 0.05
        elif abs(current_error) > 0:
            target_v = self.base_speed * 0.4
        else:
            target_v = self.base_speed

        # 5. BỘ LỌC LÀM MƯỢT (EMA)
        self.current_v = (1.0 - self.alpha_v) * self.current_v + self.alpha_v * target_v
        self.current_w = (1.0 - self.alpha_w) * self.current_w + self.alpha_w * target_w

        msg.linear.x = self.current_v
        msg.angular.z = self.current_w

        self.pub_cmd.publish(msg)
        self.pub_debug.publish(String(data=f"{debug_msg}|Err:{current_error:.2f}|Spd:{self.current_v:.2f}|L:{self.sensor_left}|R:{self.sensor_right}"))

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()