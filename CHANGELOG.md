# 更新日志

本项目遵循 [语义化版本](https://semver.org/lang/zh-CN/) 规范。

---

## [0.2.0] - 2024

### 新增
- `ArmGraspNode`（`arm_grasp`）：检测 + IK + 舵机直驱一体节点
  - HSV 颜色检测（支持 red/green/blue/yellow/orange）
  - 俯视相机像素坐标 → 基座 3D 坐标转换
  - 4-DOF 解析逆运动学（肘部向下构型，腕部保持垂直）
  - 完整状态机（SEARCH → APPROACH → DESCEND → GRASP → LIFT → PLACE → RELEASE → HOME）
  - 输出 `/servo_commands`（Int32MultiArray 0–180°），直连 ESP32
  - 运行时颜色切换（`/set_target_color`）
  - 调试画面含 HUD、检测轮廓、坐标、可达性标注
- `ArmDashboardNode`（`arm_dashboard`）：Flask Web 仪表盘
  - MJPEG 实时视频流
  - 颜色选择按钮（5 种颜色）
  - 开始 / 停止抓取控制
  - 状态机实时显示（500ms 轮询）
  - REST API 供外部集成
- `arm_grasp.launch.py`：一键启动（摄像头 + 抓取节点 + 仪表盘）
- `config/arm_grasp.yaml`：ArmGraspNode 全部参数（含注释）

### 改进
- 固件安全：提取 WiFi 凭据到 `config.h`（已加入 `.gitignore`），新增 `config.h.example`
- `setup.py`：注册 `launch/` 和 `config/` 到安装路径，修正维护者信息
- 扩充 `.gitignore`：覆盖模型文件、ROS bag、IDE 配置等

---

## [0.1.0] - 2024

### 新增
- `my_arm_vision` ROS2 视觉抓取包
  - `ObjectDetector` 节点：基于 HSV 颜色的多色物体检测
  - `YOLODetector` 节点：YOLOv8 通用物体检测
  - `SimpleVisualGrasp` 节点：内置 IK 的轻量级视觉抓取
  - `VisualGraspController` 节点：MoveIt 集成的完整抓取控制器
  - `HandEyeCalibration` 节点：Eye-to-Hand 手眼标定
- ESP32-S3 micro-ROS 固件（PCA9685 舵机控制）
- `visual_grasp.launch.py` 一键启动文件
- 参数配置文件（`config/`）
- 硬件原理图（ESP32-S3）
- YOLOv8n 模型下载脚本
