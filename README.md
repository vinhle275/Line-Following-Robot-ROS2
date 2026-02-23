# Line-Following-Robot-ROS2

Dự án robot dò line được phát triển trên nền tảng ROS 2 Jazzy và tích hợp mô phỏng với Gazebo Harmonic.

## Cấu trúc dự án

Dự án được tổ chức theo chuẩn không gian làm việc (workspace) của ROS 2. Mã nguồn được chia thành 3 package chính, chịu trách nhiệm cho từng chức năng riêng biệt của hệ thống:

```text
src/
├── line_bot_description/        # Chứa các tệp mô tả vật lý và cấu hình của robot
│   ├── urdf/                    # Các file định nghĩa khung gầm, bánh xe, cảm biến sonar và cảm biến line
│   ├── rviz/
|   ├── resource/    
│   ├── CMakeLists.txt         
│   └── package.xml            
│
├── line_bot_bringup/            # Chứa các file launch để khởi chạy toàn hệ thống
│   ├── config/                  # Chứa giá trị các tham số cần thiết
│   ├── launch/                
│   │   ├── sim.launch.py        # File khởi chạy môi trường mô phỏng Gazebo Harmonic và spawn robot
│   │   └── robot_app.launch.py  # File launch tổng hợp để gọi các node điều khiển cùng lúc
│   ├── resource
│   ├── CMakeLists.txt         
│   └── package.xml            
│
└── line_bot_control/          # Chứa các node xử lý logic và điều hướng
    ├── line_bot_control/      # Mã nguồn chính của các node điều khiển
    │   ├── drivers/        
    │   ├── line_node.py       # Đọc dữ liệu và lọc nhiễu cho các sensor dò line
    │   ├── robot_brain.py     # Node điều khiển trung tâm 
    │   ├── sonar_node.py      # Đọc dữ liệu và lọc nhiễu cho các sensor sonar
    │   └── visualize_node     # Node xử lý và trực quan hóa dữ liệu từ cảm biến
    ├── resource
    ├── setup.py               
    └── package.xml
```

## Cách chạy dựa án
### Build toàn bộ workspace
```bash
colcon build
```

### Terminal 1
```bash
source install/setup.bash
ros2 launch line_bot_bringup sim.launch.py
```

### Terminal 2
```bash
source install/setup.bash
ros2 run line_bot_control visualize_node
```

### Terminal 3
```bash
source install/setup.bash
ros2 launch line_bot_bringup robot_app.launch.py
```
