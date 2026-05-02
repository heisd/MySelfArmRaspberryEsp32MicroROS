# MySelfArmRaspberryEsp32MicroROS

> 树莓派 + ESP32 + ROS2 的机械臂视觉抓取实验仓库。

## 🗂 项目速览（Note Style）

- **目标**：完成视觉识别 + 机械臂抓取联动。
- **平台**：Raspberry Pi、ESP32、ROS2。
- **核心包**：`src/my_arm_vision`。

## 📁 目录说明

- `src/`：ROS2 工作区源码（`my_arm_vision` 包）。
- `firmware/arduino/`：ESP32 Arduino 固件。
- `hardware/schematics/`：硬件原理图与接线相关图片。
- `scripts/`：辅助脚本（如 YOLO 模型下载）。
- `docs/`：项目文档与参考资料。

## 🚀 快速开始

1. 在仓库根目录编译：

   ```bash
   colcon build --symlink-install
   ```

2. 说明：`build/`、`install/`、`log/` 为 colcon 产物，不纳入版本管理。

## 📚 文档入口

- [摄像头与机械臂流程](docs/reference/cameraArm.md)
- [Arduino 说明](docs/reference/Arduino.md)
- [colcon 常用命令](docs/reference/ColconCommand.md)
- [YOLO 模型下载](docs/model-download.md)

## ✅ 建议工作流（简版）

- 先阅读 `docs/reference/cameraArm.md` 了解全链路。
- 再准备硬件与固件（Arduino + 线缆连接）。
- 最后启动 ROS2 节点并联调视觉抓取。
