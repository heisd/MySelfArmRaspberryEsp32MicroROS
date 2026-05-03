# 更新日志

本项目遵循 [语义化版本](https://semver.org/lang/zh-CN/) 规范。

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
