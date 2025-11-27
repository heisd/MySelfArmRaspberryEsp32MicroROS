# 本文是树莓派上摄像头配合机械臂来抓取物体的使用
快速测试摄像头是否正常(需在本机上使用这个)
```bash
sudo apt install cheese
cheese
```
## 1.因为我们的摄像头是免驱动的，所以不需要安装相应驱动,安装ROS2摄像头包
```bash
sudo apt install ros-jazzy-usb-cam ros-jazzy-image-transport ros-jazzy-cv-bridge
sudo apt install ros-jazzy-image-pipeline ros-jazzy-vision-opencv
```
## 2.创建视觉识别包
```bash
mkdir src
cd src
ros2 pkg create --build-type ament_python my_arm_vision --dependencies rclpy sensor_msgs geometry_msgs cv_bridge
```
## 3.编写物体检测节点，并解决Opencv库的依赖
解决opencv库的依赖,这里我们的python版本是3.12.3
```bash
    pip install opencv-python
```
<font color ="red">> sudo pip3 install python-opencv
error: externally-managed-environment

× This environment is externally managed
╰─> To install Python packages system-wide, try apt install
    python3-xyz, where xyz is the package you are trying to
    install.
    
    If you wish to install a non-Debian-packaged Python package,
    create a virtual environment using python3 -m venv path/to/venv.
    Then use path/to/venv/bin/python and path/to/venv/bin/pip. Make
    sure you have python3-full installed.
    
    If you wish to install a non-Debian packaged Python application,
    it may be easiest to use pipx install xyz, which will manage a
    virtual environment for you. Make sure you have pipx installed.
    
    See /usr/share/doc/python3.12/README.venv for more information.

note: If you believe this is a mistake, please contact your Python installation or OS distribution provider. You can override this, at the risk of breaking your Python installation or OS, by passing --break-system-packages.
hint: See PEP 668 for the detailed specification.</font>

终端出现这个问题我们直接使用虚拟环境来解决这个问题,注意虚拟环境要和src目录同级,防止污染ROS2包环境
```bash
    cd ~/Desptop/robot
    python3 -m venv Camera
    # 激活虚拟环境
    source Camera/bin/activate
```
之后再运行
```bash
    pip install opencv-python
```
注意使用虚拟环境之后就不要在使用sudo pip3了这个命令是要安装在系统下的目录
### 版本一简单的物体追踪
文件内容如下
[object_detector.py](../src/my_arm_vision/my_arm_vision/object_detector.py)
### 版本二YOLO版本的检测物体,需要处理好Yolo版本的依赖关系，更高级一点
```bash
pip install ultralytics numpy
```
终端输出这个问题:(这是一个常见的冲突)
<font color="red">ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
generate-parameter-library-py 0.5.0 requires typeguard, which is not installed.</font></br>

安装缺失的依赖
```bash
    pip install typeguard --break-system-packages
```
文件内容如下
[yolo_detector.py](../src/my_arm_vision/my_arm_vision/yolo_detector.py)

代码解释如下:</br>
dataclasses模块化：[dataclasses](./dataclasses.md)</br>
cv_bridge构建从opencv/yolo传入的numpy到ros系统接收的sensor_msgs/Image:[cv_bridge](./cv_bridge.md)</br>



## 下面就开始编写手眼标定节点，因为我们要通过的摄像头的位置来获得末端坐标的位置，需要手眼标定来对这两个坐标进行转换
文件内容如下
[hand_eye_calibration](../src/my_arm_vision/my_arm_vision/hand_eye_calibration.py)
代码解释如下:</br>
## 下面就开始编写视觉识别抓取器
文件内容如下
[visual_grasp_controlled](../src/my_arm_vision/my_arm_vision/visual_grasp_controller.py)
代码解释如下:</br>
## 下面把这几个文件给他综合写一个launch文件
文件内容如下
[launch](../src/my_arm_vision/launch/visual_grasp.launch.py)
下面是对该文件的详解
- Node
```python
    Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
            output='screen'
        ),
```
等价于终端上运行
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev/ttyUSB0 -b 115200
```
output =‘screen'
就是把结果输出到终端上面
下面的Node(
    ...
),等都是这个意思
- remappings
```python
remappings=[
                ('image_raw', '/camera/image_raw'),
            ]
```
重映射将前面的话题名字->后面的话题名(更改为我们想要的话题名字)
- parameters(常见的参数就不多介绍)
```bash
    # 1.列出所有的视频设备
    sudo v4l2-ctl --list-devices
    # 2.找到我们的摄像头
    WebCamera: WebCamera (usb-xhci-hcd.0-2):
        /dev/video0 #摄像头
        /dev/video1 #麦克风
        /dev/media3
    # 3.检测摄像头支持的格式
    sudo v4l2-ctl --list-formats-ext -d /dev/video0
    # output
    ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 1280x960
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 800x600
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 640x360
                        Interval: Discrete 0.033s (30.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1280x960
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 800x600
                        Interval: Discrete 0.050s (20.000 fps)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 640x360
                        Interval: Discrete 0.033s (30.000 fps)    
```
根据这个输出选择格式为 mjpeg 的格式
```python
'pixel_format':'mjpeg'
```
- TimeAction :ROS2中的延时启动功能,防止这些部分缺少依赖,period是等待的时长
```python
TimeAction(
    period=2.0,
    action=[
        Node(
            ...
        ),
    ]
)
```
这样可以确保各个节点按正确顺序启动，避免因依赖未就绪而报错。
## 如果感觉上面的有点繁琐可以先试一下demo可不可以运行起来
文件内容如下
[demo](../src/my_arm_vision/my_arm_vision/demo.py)










