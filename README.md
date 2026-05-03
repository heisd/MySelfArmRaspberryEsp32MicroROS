# MySelfArm — 树莓派 × ESP32 × ROS2 视觉抓取机械臂

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205B-red.svg)
![micro-ROS](https://img.shields.io/badge/micro--ROS-ESP32--S3-orange.svg)

基于 **树莓派 5B + ESP32-S3 + ROS2 Jazzy** 的桌面机械臂视觉抓取系统。  
摄像头实时识别目标物体，通过 micro-ROS 驱动 PCA9685 舵机控制器完成抓取动作。

---

## 硬件组成

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控计算机 | Raspberry Pi 5B | 运行 ROS2，负责感知与规划 |
| 微控制器 | ESP32-S3 | 运行 micro-ROS，驱动舵机 |
| 舵机控制器 | PCA9685 | 16 通道 PWM，I²C 接口（GPIO 8/9） |
| 摄像头 | USB 摄像头 | 640×480，30fps，MJPEG |

硬件原理图：[`hardware/schematics/ESP32-S3-SCH.jpg`](hardware/schematics/ESP32-S3-SCH.jpg)

---

## 系统架构

```
[USB Camera]
     │ /camera/image_raw
     ▼
┌─────────────────────────────────┐
│         Raspberry Pi 5B         │
│                                 │
│  ObjectDetector / YOLODetector  │
│          │ /target_object_pose  │
│          ▼                      │
│  VisualGraspController          │
│          │ /joint_commands      │
└──────────┼──────────────────────┘
           │ WiFi (micro-ROS)
           ▼
     [ESP32-S3]
           │ I²C
           ▼
      [PCA9685]
           │ PWM
           ▼
       [Servos]
```

---

## 目录结构

```
MySelfArmRaspberryEsp32MicroROS/
├── firmware/
│   └── arduino/
│       ├── Arduino.ino          # ESP32-S3 micro-ROS 固件
│       └── config.h.example     # WiFi / Agent 配置模板
├── hardware/
│   └── schematics/              # 硬件原理图
├── scripts/
│   └── download_yolov8n.py      # YOLOv8n 模型下载
├── src/
│   └── my_arm_vision/           # ROS2 视觉抓取包
│       ├── config/              # 节点参数配置（YAML）
│       ├── launch/              # 启动文件
│       ├── my_arm_vision/       # Python 源码
│       └── README.md
├── docs/
│   ├── model-download.md
│   └── reference/               # 技术参考文档
├── CHANGELOG.md
├── LICENSE
└── README.md
```

---

## 快速开始

### 环境要求

- **树莓派**：ROS2 Jazzy，Ubuntu 24.04 / Raspberry Pi OS
- **ESP32-S3**：Arduino IDE + micro-ROS Arduino 库

### 1. 克隆仓库

```bash
git clone https://github.com/heisd/MySelfArmRaspberryEsp32MicroROS.git
cd MySelfArmRaspberryEsp32MicroROS
```

### 2. 安装 ROS2 依赖

```bash
sudo apt install ros-jazzy-usb-cam ros-jazzy-cv-bridge \
                 ros-jazzy-tf2-ros ros-jazzy-rqt-image-view \
                 python3-opencv python3-numpy

# YOLO 支持（可选）
pip install ultralytics --break-system-packages

# Web 仪表盘支持
pip install flask --break-system-packages
```

### 3. 下载 YOLOv8 模型（可选）

```bash
python3 scripts/download_yolov8n.py
```

### 4. 构建 ROS2 包

```bash
colcon build --symlink-install
source install/setup.bash
```

### 5. 烧录 ESP32-S3 固件

```bash
# 复制配置文件并填入实际 WiFi / Agent 信息
cp firmware/arduino/config.h.example firmware/arduino/config.h
# 编辑 config.h，然后用 Arduino IDE 烧录 Arduino.ino
```

### 6. 启动系统

```bash
# 启动 micro-ROS Agent（串口）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# 另开终端，一键启动完整视觉抓取系统
ros2 launch my_arm_vision visual_grasp.launch.py

# 可选：指定摄像头设备
ros2 launch my_arm_vision visual_grasp.launch.py camera_device:=/dev/video1
```

---

## 参数配置

所有节点参数集中在 `src/my_arm_vision/config/`：

| 文件 | 说明 |
|------|------|
| `object_detector.yaml` | 颜色检测参数（颜色列表、面积范围） |
| `visual_grasp.yaml` | 抓取控制器参数（接近距离、提起高度、放置位置） |
| `camera.yaml` | 相机内参（标定后填入） |

修改后重新构建或直接编辑（`--symlink-install` 模式下立即生效）。

---

## 功能模块

| 模块 | 说明 |
|------|------|
| `ObjectDetector` | 基于 HSV 颜色阈值检测指定颜色物体 |
| `YOLODetector` | YOLOv8 通用物体检测（适配树莓派的 nano 模型） |
| `SimpleVisualGrasp` | 内置解析 IK 的轻量级抓取节点，无需 MoveIt |
| `VisualGraspController` | MoveIt 集成的完整抓取控制器 |
| `HandEyeCalibration` | Eye-to-Hand 棋盘格手眼标定 |

详见 [`src/my_arm_vision/README.md`](src/my_arm_vision/README.md)。

---

## 文档

| 文档 | 说明 |
|------|------|
| [摄像头与机械臂流程](docs/reference/cameraArm.md) | 完整搭建流程 |
| [Arduino / ESP32 说明](docs/reference/Arduino.md) | 固件与硬件说明 |
| [colcon 常用命令](docs/reference/ColconCommand.md) | 构建与运行参考 |
| [树莓派扩展虚拟内存](docs/reference/RaspberryVirtualMemory.md) | 内存优化 |
| [YOLO 模型下载](docs/model-download.md) | 模型获取方式 |
| [更新日志](CHANGELOG.md) | 版本历史 |

---

## License

[MIT](LICENSE) © 2024 heisd
