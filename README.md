# MySelfArmRaspberryEsp32MicroROS

本仓库用于树莓派 + ESP32 + ROS2 的机械臂视觉抓取实验。

## 目录结构

- `src/`: ROS2 工作区源码（`my_arm_vision` 包）
- `firmware/arduino/`: ESP32 Arduino 固件
- `hardware/schematics/`: 硬件原理图与相关图片
- `scripts/`: 辅助脚本（如模型下载）
- `docs/`: 项目文档与参考资料

## 开发说明

- `build/`、`install/`、`log/` 属于 colcon 构建产物，已从版本管理中移除。
- 编译前请在仓库根目录执行：

```bash
colcon build --symlink-install
```

## 文档入口

- [摄像头与机械臂流程](docs/reference/cameraArm.md)
- [Arduino 说明](docs/reference/Arduino.md)
- [colcon 常用命令](docs/reference/ColconCommand.md)
- [YOLO 模型下载](docs/model-download.md)
