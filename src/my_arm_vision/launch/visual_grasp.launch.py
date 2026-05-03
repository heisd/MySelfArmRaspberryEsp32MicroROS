from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_arm_vision')
    detector_config = os.path.join(pkg_share, 'config', 'object_detector.yaml')
    grasp_config = os.path.join(pkg_share, 'config', 'visual_grasp.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='USB 摄像头设备路径'
        ),

        # 1. micro-ROS Agent（串口模式）
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
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg',
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
            ]
        ),

        # 3. 物体检测节点（延迟 2s 等待摄像头就绪）
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='my_arm_vision',
                    executable='object_detector',
                    parameters=[detector_config],
                    output='screen'
                ),
            ]
        ),

        # 4. 视觉抓取控制器（延迟 3s 等待检测节点就绪）
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='my_arm_vision',
                    executable='simple_visual_grasp',
                    parameters=[grasp_config],
                    output='screen'
                ),
            ]
        ),

        # 5. 调试图像查看
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/grasp_debug_image'],
        ),
    ])
