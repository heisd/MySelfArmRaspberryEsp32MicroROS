# my_arm_vision

ROS2 视觉抓取包，运行于树莓派，提供颜色检测、YOLO 检测、视觉抓取控制及手眼标定功能。

## 节点一览

| 节点 | 可执行文件 | 说明 |
|------|-----------|------|
| `ObjectDetector` | `object_detector` | 基于 HSV 颜色的物体检测 |
| `YOLODetector` | *(直接运行)* | YOLOv8 通用物体检测 |
| `SimpleVisualGrasp` | `simple_visual_grasp` | 轻量级视觉抓取（内置 IK，无需 MoveIt） |
| `VisualGraspController` | *(直接运行)* | MoveIt 集成的完整抓取控制器 |
| `HandEyeCalibration` | *(直接运行)* | Eye-to-Hand 手眼标定 |

## 依赖

**ROS2 包**
```
rclpy  sensor_msgs  geometry_msgs  std_msgs
cv_bridge  tf2_ros  tf2_geometry_msgs
usb_cam  micro_ros_agent
```

**Python 库**
```
opencv-python (python3-opencv)
numpy         (python3-numpy)
ultralytics   (YOLO 节点专用)
scipy         (手眼标定专用)
```

安装 YOLO 依赖：
```bash
pip install ultralytics --break-system-packages
```

## 构建

在仓库根目录执行：

```bash
colcon build --symlink-install --packages-select my_arm_vision
source install/setup.bash
```

## 快速启动

一键启动完整系统（micro-ROS Agent + 摄像头 + 检测 + 抓取 + 调试视图）：

```bash
ros2 launch my_arm_vision visual_grasp.launch.py
```

可选参数：

```bash
ros2 launch my_arm_vision visual_grasp.launch.py camera_device:=/dev/video0
```

## 单独运行节点

### 颜色检测节点

```bash
ros2 run my_arm_vision object_detector --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p detection_colors:="['red','green','blue']" \
  -p min_object_area:=500 \
  -p max_object_area:=50000
```

### 简化视觉抓取节点

```bash
ros2 run my_arm_vision simple_visual_grasp
```

启动后通过话题控制：

```bash
# 开始抓取
ros2 topic pub /start_grasp std_msgs/msg/Bool "data: true" --once

# 停止
ros2 topic pub /stop_grasp std_msgs/msg/Bool "data: true" --once
```

## 话题 & 服务

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 摄像头原始图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/joint_states_feedback` | `sensor_msgs/JointState` | 关节状态反馈 |
| `/target_object_pose` | `geometry_msgs/PoseStamped` | 目标物体位姿（控制器订阅） |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/detected_objects` | `geometry_msgs/PoseArray` | 所有检测到的物体位姿 |
| `/target_object_pose` | `geometry_msgs/PoseStamped` | 最优目标物体位姿 |
| `/detection_debug_image` | `sensor_msgs/Image` | 颜色检测调试图像 |
| `/yolo_detections` | `vision_msgs/Detection2DArray` | YOLO 检测结果 |
| `/grasp_debug_image` | `sensor_msgs/Image` | 抓取调试图像 |
| `/joint_commands` | `std_msgs/Float64MultiArray` | 关节角度指令 |
| `/gripper_command` | `std_msgs/Bool` | 夹爪开合指令（`true` = 关闭） |

### 服务（VisualGraspController）

| 服务 | 类型 | 说明 |
|------|------|------|
| `/start_grasp` | `std_srvs/Trigger` | 开始抓取任务 |
| `/stop_grasp` | `std_srvs/Trigger` | 停止并重置状态 |

## 关键参数

### ObjectDetector

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_topic` | `/camera/image_raw` | 图像话题 |
| `detection_colors` | `['red','green','blue']` | 检测颜色列表 |
| `min_object_area` | `500` | 最小检测面积（像素） |
| `max_object_area` | `50000` | 最大检测面积（像素） |

支持颜色：`red` / `green` / `blue` / `yellow` / `orange`

### YOLODetector

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model_path` | `yolov8n.pt` | YOLO 模型路径 |
| `confidence_threshold` | `0.5` | 置信度阈值 |
| `target_classes` | `['cup','bottle','apple','orange']` | 目标类别 |

模型下载参考 [docs/model-download.md](../../docs/model-download.md)。

### VisualGraspController

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `approach_distance` | `0.08` | 接近距离（米） |
| `grasp_height_offset` | `0.02` | 抓取高度偏移（米） |
| `lift_height` | `0.1` | 提起高度（米） |
| `place_position` | `[0.15, 0.15, 0.05]` | 放置位置 `[x, y, z]`（米） |

## 手眼标定

使用 Eye-to-Hand 配置（固定摄像头朝下俯视工作区）。

**所需材料**：棋盘格标定板（默认 6×9 内角点，方格 25mm）

**步骤**：

1. 启动标定节点：
   ```bash
   ros2 run my_arm_vision hand_eye_calibration
   ```

2. 将棋盘格放入相机视野，移动机械臂到不同位姿，通过服务触发采集（至少 **3 组**，建议 **10 组以上**）。

3. 触发标定计算，结果自动保存到：
   ```
   /home/pi/arm_ws/config/hand_eye_calibration.yaml
   ```

4. 下次启动时加载已有标定：标定节点将自动发布 `base_link → camera_link` 的静态 TF。

## 抓取状态机

`SimpleVisualGrasp` 和 `VisualGraspController` 均采用状态机驱动：

```
IDLE → SEARCHING/DETECTING → APPROACHING → GRASPING
     → LIFTING → MOVING → PLACING → RELEASING → HOMING → IDLE
```

## 调试

查看调试图像：

```bash
# 检测画面
ros2 run rqt_image_view rqt_image_view /detection_debug_image

# 抓取画面
ros2 run rqt_image_view rqt_image_view /grasp_debug_image
```

查看检测到的目标位姿：

```bash
ros2 topic echo /target_object_pose
```
