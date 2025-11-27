# 本文记录了几种常见的ROS2的包的运行指令
## 编译
```bash
    # 注意在虚拟环境中要添加colcon ignore
    touch Camera/COLCON_IGNORE
    # 在src上级目录编译
    cd ~/Desktop/robot
    # 全部编译
    colcon build --symlink-install
    # 选择包编译
    colcon build --packages-select ${package_name}
```
## 运行
```bash
    # source 工作空间
    source ./install/setup.zsh
    # 2. 启动视觉抓取系统
    ros2 launch my_arm_bringup visual_grasp.launch.py
    # 3. 在另一个终端，触发抓取
    ros2 topic pub /start_grasp std_msgs/Bool "data: true" --once
    # 4. 查看调试图像
    ros2 run rqt_image_view rqt_image_view /grasp_debug_image
```
