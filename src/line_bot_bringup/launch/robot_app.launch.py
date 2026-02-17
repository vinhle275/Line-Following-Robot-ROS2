import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Đường dẫn đến file cấu hình params.yaml
    config = os.path.join(
        get_package_share_directory('line_bot_bringup'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        # --- NODE 1: CẢM BIẾN SONAR (Mắt) ---
        Node(
            package='line_bot_control',
            executable='sonar_node',
            name='sonar_node',
            output='screen',
            parameters=[config]
        ),

        # --- NODE 2: CẢM BIẾN LINE (Dò đường) ---
        Node(
            package='line_bot_control',
            executable='line_node',
            name='line_node',
            output='screen',
            parameters=[config]
        ),
        
        # --- NODE 3: BỘ NÃO ROBOT (Xử lý PID) ---
        Node(
            package='line_bot_control',
            executable='robot_brain',
            name='robot_brain_node',
            output='screen',
            parameters=[config]
        )
    ])