# 本文是树莓派上摄像头的使用
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
解决opencv库的依赖
```bash
    sudo pip3 install python-opencv
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

终端出现这个问题我们直接使用虚拟环境来解决这个问题,注意虚拟环境要和src目录同级
```bash
    cd ~/Desptop/robot
    python3 -m venv Camera
```
### 版本一简单的物体追踪
文件内容如下
[object_detector.py](../src/my_arm_vision/my_arm_vision/object_detector.py)
### 版本二YOLO版本的检测物体,需要处理好Yolo版本的依赖关系
```bash



```
文件内容如下
[yolo_detector.py](../src/my_arm_vision/my_arm_vision/yolo_detector.py)





