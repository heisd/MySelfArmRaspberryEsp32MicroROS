# my_arm_vision

ROS2 视觉抓取包，运行于树莓派，提供颜色检测、视觉抓取控制、Web 仪表盘及手眼标定功能。

---

## 节点一览

| 节点 | 可执行文件 | 说明 |
|------|-----------|------|
| `ArmGraspNode` | `arm_grasp` | **推荐** 检测 + 4-DOF IK + 舵机直驱，直连 ESP32 |
| `ArmDashboardNode` | `arm_dashboard` | **推荐** Flask Web 仪表盘（视频流 + 颜色选择 + 控制） |
| `ObjectDetector` | `object_detector` | 独立 HSV 颜色检测节点 |
| `YOLODetector` | — | YOLOv8 通用物体检测 |
| `SimpleVisualGrasp` | `simple_visual_grasp` | 内置 IK 的轻量抓取节点 |
| `VisualGraspController` | — | MoveIt 集成的完整抓取控制器 |
| `HandEyeCalibration` | — | Eye-to-Hand 棋盘格手眼标定 |

---

## 依赖

**ROS2 包**
```
rclpy  sensor_msgs  geometry_msgs  std_msgs
cv_bridge  tf2_ros  tf2_geometry_msgs
usb_cam  micro_ros_agent
```

**Python 库**

```bash
# 必须
sudo apt install python3-opencv python3-numpy

# Web 仪表盘（arm_dashboard）
pip install flask --break-system-packages

# YOLO 检测（yolo_detector）
pip install ultralytics --break-system-packages

# 手眼标定（hand_eye_calibration）
pip install scipy --break-system-packages
```

---

## 构建

```bash
# 在仓库根目录执行
colcon build --symlink-install --packages-select my_arm_vision
source install/setup.bash
```

---

## 快速启动

### 推荐：一体化启动（抓取 + Web 仪表盘）

```bash
ros2 launch my_arm_vision arm_grasp.launch.py
```

启动后浏览器打开 `http://<树莓派IP>:5000`：
- 实时摄像头检测画面
- 点击颜色按钮切换目标
- 一键启动 / 停止抓取

```bash
# 指定摄像头和初始颜色
ros2 launch my_arm_vision arm_grasp.launch.py \
  camera_device:=/dev/video0 \
  target_color:=green
```

### 其他：原始启动文件（MoveIt 控制器）

```bash
ros2 launch my_arm_vision visual_grasp.launch.py
```

---

## ArmGraspNode — 检测抓取一体节点

将摄像头检测、坐标转换、4-DOF 解析 IK、状态机控制整合为一个节点，  
输出 `/servo_commands`（`Int32MultiArray`，0–180°），与 ESP32 Arduino 固件直接对接。

```
/camera/image_raw
        │
        ▼
  HSV 颜色检测
        │ 像素坐标
        ▼
  像素 → 基座 3D 坐标（俯视相机模型）
        │
        ▼
  4-DOF 解析 IK
        │ 关节角（度）
        ▼
/servo_commands ──WiFi──▶ ESP32 ──I²C──▶ PCA9685 ──PWM──▶ 舵机
```

### 运行

```bash
ros2 run my_arm_vision arm_grasp --ros-args \
  -p target_color:=red \
  -p cam_z:=0.40
```

### 控制话题

```bash
# 开始抓取
ros2 topic pub /start_grasp std_msgs/msg/Bool "data: true" --once

# 停止
ros2 topic pub /stop_grasp std_msgs/msg/Bool "data: true" --once

# 运行时切换颜色
ros2 topic pub /set_target_color std_msgs/msg/String "data: 'green'" --once
```

### 查看调试画面

```bash
ros2 run rqt_image_view rqt_image_view /arm_debug_image
```

调试画面内容：检测轮廓、物体中心点、3D 坐标估算、可达性标注、当前状态 HUD。

### 抓取状态机

```
IDLE
 └─▶ SEARCHING       等待检测到目标且在工作空间内
       └─▶ APPROACHING    移动到目标正上方（夹爪打开）
             └─▶ DESCENDING    下降到抓取高度
                   └─▶ GRASPING     关闭夹爪
                         └─▶ LIFTING      提起物体
                               └─▶ MOVING_TO_PLACE  移向放置区上方
                                     └─▶ PLACING     下降至放置高度
                                           └─▶ RELEASING  打开夹爪
                                                 └─▶ HOMING  归位
                                                       └─▶ IDLE ✓
```

### 关键参数（`config/arm_grasp.yaml`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_color` | `red` | 检测颜色（red/green/blue/yellow/orange） |
| `cam_z` | `0.40` | 相机距桌面高度（米） |
| `cam_y` | `0.20` | 相机在基座前方的距离（米） |
| `link_base` | `0.08` | 底座到肩关节高度（米） |
| `link1 / link2 / link3` | `0.10 / 0.10 / 0.06` | 大臂 / 小臂 / 腕部长度（米） |
| `joint_offsets` | `[90, 90, 90, 90]` | 各舵机零点偏移（度） |
| `gripper_channel` | `4` | 夹爪 PCA9685 通道号 |
| `gripper_open / close` | `90 / 20` | 夹爪开合角度（度） |
| `place_x / place_y` | `0.15 / 0.12` | 放置目标坐标（米） |
| `approach_height` | `0.08` | 接近阶段在目标上方的高度（米） |
| `lift_height` | `0.10` | 提起阶段离地高度（米） |
| `move_duration` | `2.0` | 单段运动等待时长（秒） |

> **首次使用前必须校准 `joint_offsets`**：  
> 将机械臂手动摆到 home 姿态，观察各舵机当前角度，  
> 计算 `offset = 期望输出90° - IK计算角度`，填入 YAML。

---

## ArmDashboardNode — Web 仪表盘

```bash
ros2 run my_arm_vision arm_dashboard
# 浏览器打开：http://<IP>:5000
```

### 功能

| 功能 | 实现方式 |
|------|---------|
| 实时视频流 | MJPEG `/arm_debug_image`，含检测框和状态 HUD |
| 颜色选择 | 按钮 → POST `/api/color/<color>` → 发布 `/set_target_color` |
| 开始 / 停止 | 按钮 → POST `/api/start` / `/api/stop` → 发布 Bool 话题 |
| 状态显示 | 轮询 `GET /api/status`（500ms），订阅 `/arm_state` |

### REST API

| 路径 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 仪表盘页面 |
| `/video_feed` | GET | MJPEG 视频流 |
| `/api/status` | GET | `{"state":"SEARCHING","color":"red"}` |
| `/api/start` | POST | 开始抓取 |
| `/api/stop` | POST | 停止 |
| `/api/color/<color>` | POST | 切换目标颜色 |

---

## 话题参考

### 订阅

| 话题 | 类型 | 节点 |
|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 全部视觉节点 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 自动更新相机内参 |
| `/start_grasp` | `std_msgs/Bool` | ArmGraspNode / SimpleVisualGrasp |
| `/stop_grasp` | `std_msgs/Bool` | ArmGraspNode / SimpleVisualGrasp |
| `/set_target_color` | `std_msgs/String` | ArmGraspNode |
| `/target_object_pose` | `geometry_msgs/PoseStamped` | VisualGraspController |
| `/joint_states_feedback` | `sensor_msgs/JointState` | SimpleVisualGrasp |

### 发布

| 话题 | 类型 | 节点 |
|------|------|------|
| `/servo_commands` | `std_msgs/Int32MultiArray` | ArmGraspNode → ESP32 |
| `/arm_debug_image` | `sensor_msgs/Image` | ArmGraspNode |
| `/arm_state` | `std_msgs/String` | ArmGraspNode |
| `/detected_objects` | `geometry_msgs/PoseArray` | ObjectDetector |
| `/target_object_pose` | `geometry_msgs/PoseStamped` | ObjectDetector / YOLODetector |
| `/detection_debug_image` | `sensor_msgs/Image` | ObjectDetector |
| `/yolo_detections` | `vision_msgs/Detection2DArray` | YOLODetector |
| `/joint_commands` | `std_msgs/Float64MultiArray` | SimpleVisualGrasp |
| `/gripper_command` | `std_msgs/Bool` | SimpleVisualGrasp |

---

## 独立运行其他节点

### 颜色检测节点

```bash
ros2 run my_arm_vision object_detector --ros-args \
  -p detection_colors:="['red','green','blue']" \
  -p min_object_area:=500
```

### 简化抓取节点（无 MoveIt，无 Dashboard）

```bash
ros2 run my_arm_vision simple_visual_grasp
# 控制
ros2 topic pub /start_grasp std_msgs/msg/Bool "data: true" --once
```

### YOLO 检测

```bash
# 先下载模型
python3 scripts/download_yolov8n.py
# 再运行（参见 yolo_detector.py）
```

---

## 手眼标定

Eye-to-Hand 配置（固定摄像头俯视工作台）。

**所需材料**：棋盘格标定板（默认 6×9 内角点，方格边长 25mm）

```bash
ros2 run my_arm_vision hand_eye_calibration
```

步骤：
1. 将棋盘格放入视野，移动机械臂到不同位姿
2. 每次通过服务/话题触发采集（至少 **3 组**，建议 **10 组以上**）
3. 触发标定计算 → 结果保存到 `src/my_arm_vision/config/hand_eye_calibration.yaml`
4. 重启后自动发布 `base_link → camera_link` 静态 TF

---

## 调试

```bash
# 查看检测 + 抓取画面（推荐）
ros2 run rqt_image_view rqt_image_view /arm_debug_image

# 查看原始颜色检测画面
ros2 run rqt_image_view rqt_image_view /detection_debug_image

# 监听当前状态
ros2 topic echo /arm_state

# 监听舵机指令
ros2 topic echo /servo_commands

# 监听目标位姿
ros2 topic echo /target_object_pose
```
