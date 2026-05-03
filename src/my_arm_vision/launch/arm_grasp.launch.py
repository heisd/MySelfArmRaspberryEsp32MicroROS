from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share  = get_package_share_directory('my_arm_vision')
    grasp_cfg  = os.path.join(pkg_share, 'config', 'arm_grasp.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='USB 摄像头设备路径'
        ),
        DeclareLaunchArgument(
            'target_color',
            default_value='red',
            description='初始检测颜色: red/green/blue/yellow/orange'
        ),

        # 1. micro-ROS Agent（串口，驱动 ESP32）
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
            output='screen'
        ),

        # 2. USB 摄像头
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[{
                'video_device': LaunchConfiguration('camera_device'),
                'image_width':  640,
                'image_height': 480,
                'framerate':    30.0,
                'pixel_format': 'mjpeg',
            }],
            remappings=[('image_raw', '/camera/image_raw')]
        ),

        # 3. 物体检测 + 抓取控制节点（延迟 2s 等待摄像头）
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='my_arm_vision',
                    executable='arm_grasp',
                    parameters=[
                        grasp_cfg,
                        {'target_color': LaunchConfiguration('target_color')},
                    ],
                    output='screen'
                ),
            ]
        ),

        # 4. Web 仪表盘（延迟 2.5s）
        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='my_arm_vision',
                    executable='arm_dashboard',
                    parameters=[grasp_cfg],
                    output='screen'
                ),
            ]
        ),
    ])
