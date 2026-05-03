# MySelfArm — 树莓派 × ESP32 × ROS2 视觉抓取机械臂

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205B-red.svg)
![micro-ROS](https://img.shields.io/badge/micro--ROS-ESP32--S3-orange.svg)

基于 **树莓派 5B + ESP32-S3 + ROS2 Jazzy** 的桌面机械臂视觉抓取系统。  
摄像头实时识别目标物体颜色，通过 micro-ROS WiFi 驱动 PCA9685 舵机控制器完成抓取动作。  
内置 **Web 仪表盘**，浏览器即可实时查看画面并控制机械臂。

---

## 硬件组成

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控计算机 | Raspberry Pi 5B | 运行 ROS2，负责感知与规划 |
| 微控制器 | ESP32-S3 | 运行 micro-ROS，驱动舵机 |
| 舵机控制器 | PCA9685 | 16 通道 PWM，I²C 接口（SDA=GPIO8 / SCL=GPIO9） |
| 摄像头 | USB 摄像头 | 640×480，30fps，MJPEG |

硬件原理图：[`hardware/schematics/ESP32-S3-SCH.jpg`](hardware/schematics/ESP32-S3-SCH.jpg)

---

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5B                        │
│                                                           │
│  [USB Camera] ──/camera/image_raw──▶ ArmGraspNode        │
│                                          │                │
│                                   HSV 颜色检测            │
│                                   像素 → 基座坐标         │
│                                   4-DOF 解析 IK           │
│                                          │                │
│                             /servo_commands (0–180°)      │
│                                          │                │
│  [Browser] ◀──── ArmDashboardNode ◀─────┤                │
│  http://<IP>:5000   (Flask MJPEG)   /arm_state            │
└──────────────────────────────────────────┼───────────────┘
                                           │ WiFi (micro-ROS)
                                           ▼
                                      [ESP32-S3]
                                           │ I²C
                                           ▼
                                      [PCA9685]
                                           │ PWM ×16
                                           ▼
                                        [舵机]
```

---

## 目录结构

```
MySelfArmRaspberryEsp32MicroROS/
├── firmware/
│   └── arduino/
│       ├── Arduino.ino          # ESP32-S3 micro-ROS 固件
│       └── config.h.example     # WiFi / Agent 配置模板（复制为 config.h 后填写）
├── hardware/
│   └── schematics/              # 硬件原理图
├── scripts/
│   └── download_yolov8n.py      # YOLOv8n 模型下载脚本
├── src/
│   └── my_arm_vision/           # ROS2 视觉抓取包
│       ├── config/              # 节点参数配置（YAML）
│       ├── launch/              # 启动文件
│       ├── my_arm_vision/       # Python 源码
│       └── README.md            # 包级详细文档
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

| 环境 | 要求 |
|------|------|
| 操作系统 | Ubuntu 24.04 / Raspberry Pi OS（64-bit） |
| ROS2 | Jazzy |
| Python | 3.10+ |
| ESP32 固件 | Arduino IDE + micro-ROS Arduino 库 |

### 1. 克隆仓库

```bash
git clone https://github.com/heisd/MySelfArmRaspberryEsp32MicroROS.git
cd MySelfArmRaspberryEsp32MicroROS
```

### 2. 安装 ROS2 系统依赖

```bash
sudo apt install ros-jazzy-usb-cam ros-jazzy-cv-bridge \
                 ros-jazzy-tf2-ros ros-jazzy-rqt-image-view \
                 python3-colcon-common-extensions
```

### 3. 创建虚拟环境并安装 Python 依赖

> **必须使用 `--system-site-packages`**，否则虚拟环境无法访问 ROS2 系统包（rclpy 等）。

```bash
# 方式一：一键脚本
bash scripts/setup_venv.sh

# 方式二：手动
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

激活虚拟环境（每次开新终端时执行）：

```bash
source .venv/bin/activate
```

### 5. 构建 ROS2 包

```bash
colcon build --symlink-install
source install/setup.bash
```

### 6. 烧录 ESP32-S3 固件

```bash
# 从模板创建配置文件，填入真实 WiFi 和树莓派 IP
cp firmware/arduino/config.h.example firmware/arduino/config.h
# 用 Arduino IDE 打开并烧录 firmware/arduino/Arduino.ino
```

### 7. 下载 YOLO 模型（可选）

```bash
python3 scripts/download_yolov8n.py
```

### 8. 启动系统

```bash
# 一键启动：摄像头 + 检测抓取节点 + Web 仪表盘
ros2 launch my_arm_vision arm_grasp.launch.py

# 可选参数
ros2 launch my_arm_vision arm_grasp.launch.py \
  camera_device:=/dev/video0 \
  target_color:=red
```

> 启动后在浏览器打开 **`http://<树莓派IP>:5000`** 即可进入控制仪表盘。

---

## Web 仪表盘

| 功能 | 说明 |
|------|------|
| 实时视频流 | MJPEG 画面，显示检测框、3D 坐标、可达性、当前状态 |
| 颜色选择 | 点击按钮切换目标颜色（红 / 绿 / 蓝 / 黄 / 橙） |
| 开始 / 停止 | 一键触发抓取任务 |
| 状态显示 | 实时显示状态机当前阶段（500ms 刷新） |

REST API 也可供外部程序调用：

```
GET  /api/status          → {"state": "SEARCHING", "color": "red"}
POST /api/start           → 开始抓取
POST /api/stop            → 停止
POST /api/color/<color>   → 切换颜色
GET  /video_feed          → MJPEG 流
```

---

## 功能模块

| 模块 | 可执行文件 | 说明 |
|------|-----------|------|
| `ArmGraspNode` | `arm_grasp` | **推荐** 检测 + IK + 舵机直驱，输出直连 ESP32 |
| `ArmDashboardNode` | `arm_dashboard` | **推荐** Flask Web 仪表盘 |
| `ObjectDetector` | `object_detector` | 独立 HSV 颜色检测节点 |
| `YOLODetector` | — | YOLOv8 通用物体检测 |
| `SimpleVisualGrasp` | `simple_visual_grasp` | 内置 IK 的轻量抓取节点 |
| `VisualGraspController` | — | MoveIt 集成的完整抓取控制器 |
| `HandEyeCalibration` | — | Eye-to-Hand 棋盘格手眼标定 |

详见 [`src/my_arm_vision/README.md`](src/my_arm_vision/README.md)。

---

## 参数配置

所有节点参数集中在 `src/my_arm_vision/config/`，`--symlink-install` 构建后修改 YAML 立即生效：

| 文件 | 说明 |
|------|------|
| `arm_grasp.yaml` | ArmGraspNode 全部参数（颜色、相机、IK、舵机偏移） |
| `object_detector.yaml` | 独立颜色检测节点参数 |
| `visual_grasp.yaml` | MoveIt 抓取控制器参数 |
| `camera.yaml` | 相机内参模板（标定后填入） |

### 首次使用必须调整的参数（`arm_grasp.yaml`）

```yaml
# 机械臂连杆长度（米）——按实际 URDF 修改
link_base: 0.08
link1: 0.10
link2: 0.10
link3: 0.06

# 舵机零点偏移（度）——让 home 姿态时所有舵机输出 90°
joint_offsets: [90, 90, 90, 90]

# 相机安装高度（米）
cam_z: 0.40
```

---

## 文档

| 文档 | 说明 |
|------|------|
| [包级详细文档](src/my_arm_vision/README.md) | 所有节点、话题、参数、标定流程 |
| [摄像头与机械臂流程](docs/reference/cameraArm.md) | 完整搭建流程 |
| [Arduino / ESP32 说明](docs/reference/Arduino.md) | 固件与硬件说明 |
| [colcon 常用命令](docs/reference/ColconCommand.md) | 构建与运行参考 |
| [树莓派扩展虚拟内存](docs/reference/RaspberryVirtualMemory.md) | 内存优化 |
| [YOLO 模型下载](docs/model-download.md) | 模型获取方式 |
| [更新日志](CHANGELOG.md) | 版本历史 |

---

## License

[MIT](LICENSE) © 2024 heisd
