import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float32, String
import tkinter as tk
from tkinter import ttk
import threading

class VisualizeNode(Node):
    def __init__(self, update_callbacks):
        super().__init__('visualize_node')
        self.update_callbacks = update_callbacks

        # --- Subscriptions ---
        self.create_subscription(Int8, '/sensors/line_left', self.cb_line_left, 10)
        self.create_subscription(Int8, '/sensors/line_right', self.cb_line_right, 10)
        self.create_subscription(Float32, '/sensors/sonar/filtered', self.cb_sonar, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)
        self.create_subscription(String, '/robot/debug_state', self.cb_state, 10)

        self.get_logger().info('Visualize Node with GUI started!')

    def cb_line_left(self, msg): self.update_callbacks['line_left'](msg.data)
    def cb_line_right(self, msg): self.update_callbacks['line_right'](msg.data)
    def cb_sonar(self, msg): self.update_callbacks['sonar'](msg.data)
    def cb_cmd(self, msg): self.update_callbacks['cmd'](msg.linear.x, msg.angular.z)
    def cb_state(self, msg): self.update_callbacks['state'](msg.data)


class DashboardGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Robot Debug Dashboard")
        self.root.geometry("450x350")
        self.root.configure(padx=20, pady=20)
        
        style = ttk.Style()
        style.configure("TLabel", font=("Consolas", 11))
        style.configure("Header.TLabel", font=("Consolas", 13, "bold"), foreground="#333333")
        
        ttk.Label(root, text="📡 SENSORS", style="Header.TLabel").pack(anchor="w", pady=(0, 5))
        self.lbl_line_left = ttk.Label(root, text="Mắt trái (Line)  : Chờ dữ liệu...")
        self.lbl_line_left.pack(anchor="w")
        
        self.lbl_line_right = ttk.Label(root, text="Mắt phải (Line)  : Chờ dữ liệu...")
        self.lbl_line_right.pack(anchor="w")
        
        self.lbl_sonar = ttk.Label(root, text="Khoảng cách Sonar: Chờ dữ liệu...")
        self.lbl_sonar.pack(anchor="w", pady=(0, 15))

        ttk.Label(root, text="🚀 VELOCITY", style="Header.TLabel").pack(anchor="w", pady=(0, 5))
        self.lbl_vel_x = ttk.Label(root, text="Tốc độ tiến (X)  : 0.00 m/s")
        self.lbl_vel_x.pack(anchor="w")
        
        self.lbl_vel_z = ttk.Label(root, text="Tốc độ xoay (Z)  : 0.00 rad/s")
        self.lbl_vel_z.pack(anchor="w", pady=(0, 15))
        
        ttk.Label(root, text="🧠 BRAIN STATE", style="Header.TLabel").pack(anchor="w", pady=(0, 5))
        self.lbl_state = ttk.Label(root, text="Trạng thái: WAITING...", foreground="blue", wraplength=400)
        self.lbl_state.pack(anchor="w")

    # Các hàm cập nhật text trên cửa sổ
    def update_line_left(self, val):
        status = "🔴 [ 1 ] Lệch line" if val == 1 else "🟢 [ 0 ] An toàn"
        self.lbl_line_left.config(text=f"Mắt trái (Line)  : {status}")

    def update_line_right(self, val):
        status = "🔴 [ 1 ] Lệch line" if val == 1 else "🟢 [ 0 ] An toàn"
        self.lbl_line_right.config(text=f"Mắt phải (Line)  : {status}")

    def update_sonar(self, val):
        self.lbl_sonar.config(text=f"Khoảng cách Sonar: {val:.2f} m")

    def update_cmd(self, x, z):
        self.lbl_vel_x.config(text=f"Tốc độ tiến (X)  : {x:.2f} m/s")
        self.lbl_vel_z.config(text=f"Tốc độ xoay (Z)  : {z:.2f} rad/s")

    def update_state(self, val):
        self.lbl_state.config(text=f"Trạng thái: {val}")


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    gui = DashboardGUI(root)
    
    callbacks = {
        'line_left': gui.update_line_left,
        'line_right': gui.update_line_right,
        'sonar': gui.update_sonar,
        'cmd': gui.update_cmd,
        'state': gui.update_state
    }
    
    node = VisualizeNode(callbacks)
        
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()