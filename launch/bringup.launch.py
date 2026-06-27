import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('robot_mino_control')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot_mino.urdf.xacro')
    controllers_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    slam_path = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    # 1. PATH KE FILE CONFIG EKF (Pastikan file ekf.yaml ditaruh di folder config package Anda)
    ekf_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Mengonversi xacro ke XML secara sinkron
    doc = xacro.process_file(xacro_path)
    robot_description_content = doc.toxml()
    robot_description = {'robot_description': robot_description_content}

    # Node 1: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Node 2: Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_path],
    )

    # Node 3: Serial Bridge Node
    serial_bridge_node = Node(
        package='robot_mino_control',
        executable='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'counts_per_rev': 1012.0,
        }],
    )

    # Node 4: Custom Lidar Node
    lidar_node = Node(
        package='robot_mino_control', 
        executable='lidar_node.py',   
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'baud_rate': 115200,
            'frame_id': 'lidar_link',
        }],
    )

    # 2. NODE EKF (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_path], # Menggunakan file ekf.yaml
    )

    # Node 5: Spawner Joint State Broadcaster
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Node 6: Spawner Diff Drive Controller
    spawner_ddc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # Delay Handler: Menahan Spawner sampai Controller Manager siap
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[spawner_jsb, spawner_ddc],
        )
    )

    # Tambahkan di daftar return [ ... ] dalam launch_setup:
    slam_node = Node(
        
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_path,
            {'use_sim_time': False}
        ]
    )

    return [
        robot_state_publisher_node,
        controller_manager_node,
        serial_bridge_node,
        lidar_node,
        ekf_node,          # <-- Masukkan Node EKF ke dalam daftar eksekusi
        slam_node,
        delay_spawners, 
    ]


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Port serial Arduino/ESP32 (ros2_control)'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='Port serial untuk Lidar'
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        OpaqueFunction(function=launch_setup)
    ])