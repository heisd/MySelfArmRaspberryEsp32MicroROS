from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 参数
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        
        # 1. micro-ROS Agent serial mode 
        # 在终端上运行ros2 run micro_ros_agent micro_ros_agent serial --dev/ttyUSB0 -b 115200
        
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
                'pixel_format': 'yuyv',
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
            ]
        ),
        
        # 3. 物体检测节点
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='my_arm_vision',
                    executable='object_detector',
                    parameters=[{
                        'camera_topic': '/camera/image_raw',
                        'detection_colors': ['red', 'green', 'blue'],
                        'min_object_area': 500,
                    }],
                    output='screen'
                ),
            ]
        ),
        
        # 4. 视觉抓取控制器
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='my_arm_vision',
                    executable='simple_visual_grasp',
                    output='screen'
                ),
            ]
        ),
        
        # 5. RViz (可选)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        # ),
        
        # 6. rqt_image_view 查看调试图像
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/grasp_debug_image'],
        ),
    ])