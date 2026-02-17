import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('line_bot_bringup')
    pkg_description = get_package_share_directory('line_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file_path = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    world_path = os.path.join(pkg_bringup, 'worlds', 'track.sdf')
    
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_bringup, 'worlds')
    )

    # 1. Chạy Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Robot State Publisher
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # 3. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', '0.0', '-y', '-2.0', '-z', '0.2', '-Y', '0.0'
        ],
        output='screen'
    )

    # 4. BRIDGE (Cầu nối trực tiếp - Không dùng YAML)
    # Chúng ta ép Bridge chạy chế độ 'best_effort' cho các sensor
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock & Control
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            # SENSORS (Tên topic ngắn gọn từ URDF)
            '/sensors/line_left_raw@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/sensors/line_right_raw@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/sensors/sonar_front_raw@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        parameters=[{
            # Ép ROS Publisher phải là Best Effort để khớp với Gazebo
            'qos_overrides./sensors/line_left_raw.publisher.reliability': 'best_effort',
            'qos_overrides./sensors/line_right_raw.publisher.reliability': 'best_effort',
            'qos_overrides./sensors/sonar_front_raw.publisher.reliability': 'best_effort',
        }],
        output='screen'
    )

    return LaunchDescription([
        resource_env, gz_sim, robot_state_publisher, spawn_entity, bridge
    ])