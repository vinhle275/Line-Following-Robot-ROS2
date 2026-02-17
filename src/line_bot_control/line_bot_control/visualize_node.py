#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.create_subscription(String, '/robot/debug_state', self.listener_cb, 10)
        self.current_state = "Waiting..."
        self.error_val = 0.0
        self.speed = 0.0
        
        # Biến lưu trạng thái sensor và dữ liệu chi tiết
        self.is_left_on = False
        self.is_right_on = False
        self.left_sensor = 0.0
        self.right_sensor = 0.0
        
        self.create_timer(0.1, self.draw_loop)

    def listener_cb(self, msg):
        # Format msg: "State|Err:x|Spd:y|L:1|R:0"
        try:
            parts = msg.data.split('|')
            self.current_state = parts[0]
            self.error_val = float(parts[1].split(':')[1])
            self.speed = float(parts[2].split(':')[1])
            
            # Đọc trạng thái sensor từ chuỗi debug
            if len(parts) >= 5:
                self.is_left_on = int(parts[3].split(':')[1]) == 1
                self.is_right_on = int(parts[4].split(':')[1]) == 1
        except:
            pass

    def draw_loop(self):
        img = np.zeros((500, 800, 3), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # --- TIÊU ĐỀ ---
        cv2.putText(img, "LINE FOLLOWER ROBOT - DASHBOARD", (30, 40), font, 1.2, (255, 255, 255), 2)
        
        # --- TRạNG THÁI CHÍNH ---
        state_color = (0, 255, 0) if self.current_state not in ["LOST", "SEARCH"] else (0, 0, 255)
        cv2.putText(img, f"STATE: {self.current_state}", (30, 80), font, 1, state_color, 2)
        
        # --- THÔNG TIN TỐC ĐỘ VÀ LỖI ---
        cv2.putText(img, f"Speed: {self.speed:.3f} m/s", (30, 120), font, 0.9, (0, 255, 255), 1)
        cv2.putText(img, f"Steering Error: {self.error_val:.3f}", (30, 160), font, 0.9, (0, 255, 255), 1)
        
        # --- VẼ CỬA SỔ THÔNG TIN CẢM BIẾN ---
        # Khung cho cảm biến trái
        cv2.rectangle(img, (30, 200), (350, 320), (200, 200, 200), 1)
        cv2.putText(img, "LEFT SENSOR", (40, 225), font, 0.8, (255, 255, 255), 1)
        
        # Khung cho cảm biến phải
        cv2.rectangle(img, (400, 200), (720, 320), (200, 200, 200), 1)
        cv2.putText(img, "RIGHT SENSOR", (410, 225), font, 0.8, (255, 255, 255), 1)
        
        # Vẽ trạng thái cảm biến (vòng tròn lớn)
        c_left = (0, 255, 0) if self.is_left_on else (0, 0, 255)
        c_right = (0, 255, 0) if self.is_right_on else (0, 0, 255)
        
        cv2.circle(img, (120, 260), 30, c_left, -1)
        cv2.circle(img, (490, 260), 30, c_right, -1)
        
        cv2.putText(img, "ON" if self.is_left_on else "OFF", (105, 270), font, 0.7, (0, 0, 0), 2)
        cv2.putText(img, "ON" if self.is_right_on else "OFF", (475, 270), font, 0.7, (0, 0, 0), 2)
        
        # Vẽ mũi tên hướng rẽ
        arrow_y = 360
        cv2.putText(img, "STEERING DIRECTION:", (30, arrow_y), font, 0.9, (255, 255, 255), 1)
        
        if abs(self.error_val) < 0.15:  # Đi thẳng
            cv2.arrowedLine(img, (400, arrow_y + 50), (400, arrow_y - 20), (0, 255, 0), 4)
            cv2.putText(img, "STRAIGHT", (370, arrow_y + 80), font, 0.8, (0, 255, 0), 1)
        elif self.error_val > 0.15:  # Rẽ trái
            cv2.arrowedLine(img, (400, arrow_y + 50), (280, arrow_y + 50), (255, 0, 0), 4)
            cv2.putText(img, "LEFT", (310, arrow_y + 80), font, 0.8, (255, 0, 0), 1)
        else:  # Rẽ phải
            cv2.arrowedLine(img, (400, arrow_y + 50), (520, arrow_y + 50), (255, 0, 0), 4)
            cv2.putText(img, "RIGHT", (490, arrow_y + 80), font, 0.8, (255, 0, 0), 1)
        
        # --- VẼ THANH HỖ TRỢ TRỰC QUAN CHO LỖI ---
        bar_y = 440
        cv2.putText(img, "ERROR RANGE:", (30, bar_y), font, 0.8, (255, 255, 255), 1)
        
        # Thanh ngang biểu diễn phạm vi lỗi
        bar_x_start = 180
        bar_width = 400
        bar_height = 30
        
        # Vẽ nền thanh
        cv2.rectangle(img, (bar_x_start, bar_y - bar_height//2), 
                      (bar_x_start + bar_width, bar_y + bar_height//2), (100, 100, 100), -1)
        
        # Vẽ vị trí giữa (0 error)
        cv2.line(img, (bar_x_start + bar_width//2, bar_y - bar_height//2 - 5),
                (bar_x_start + bar_width//2, bar_y + bar_height//2 + 5), (255, 255, 0), 2)
        
        # Vẽ vị trí hiện tại dựa trên error
        error_pos = int(bar_x_start + bar_width//2 + self.error_val * 100)  # Scale lỗi
        error_pos = max(bar_x_start, min(bar_x_start + bar_width, error_pos))
        cv2.circle(img, (error_pos, bar_y), 8, (0, 255, 255), -1)
        
        # Nhãn trái phải
        cv2.putText(img, "LEFT", (bar_x_start - 50, bar_y + 5), font, 0.6, (100, 100, 255), 1)
        cv2.putText(img, "RIGHT", (bar_x_start + bar_width + 20, bar_y + 5), font, 0.6, (100, 100, 255), 1)

        cv2.imshow("Line Following Robot - Real-time Monitor", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
