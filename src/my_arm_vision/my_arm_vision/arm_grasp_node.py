#!/usr/bin/env python3
"""
arm_grasp_node.py — 物体检测 + 机械臂抓取一体节点

检测摄像头画面中指定颜色的物体，自动驱动机械臂完成
  搜索 → 接近 → 抓取 → 搬运 → 放置 → 归位
的完整流程。

舵机指令直接发布到 /servo_commands (Int32MultiArray, 0-180°)，
与 ESP32-S3 Arduino 固件直接对接，无需 MoveIt。

话题（订阅）:
  /camera/image_raw     sensor_msgs/Image       摄像头图像
  /camera/camera_info   sensor_msgs/CameraInfo  相机内参（可选，有则自动使用）
  /start_grasp          std_msgs/Bool           True = 开始任务
  /stop_grasp           std_msgs/Bool           True = 停止任务

话题（发布）:
  /servo_commands       std_msgs/Int32MultiArray  关节角度 + 夹爪（°）
  /arm_debug_image      sensor_msgs/Image         检测调试画面
"""

import math
import time
from enum import Enum
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32MultiArray, String


# ── 颜色 HSV 范围定义 ─────────────────────────────────────────────────────────

COLOR_RANGES = {
    'red': [
        (np.array([0,   100, 100]), np.array([10,  255, 255])),
        (np.array([160, 100, 100]), np.array([180, 255, 255])),
    ],
    'green':  [(np.array([35,  100, 100]), np.array([85,  255, 255]))],
    'blue':   [(np.array([100, 100, 100]), np.array([130, 255, 255]))],
    'yellow': [(np.array([20,  100, 100]), np.array([35,  255, 255]))],
    'orange': [(np.array([10,  100, 100]), np.array([20,  255, 255]))],
}

COLOR_BGR = {
    'red':    (0,   0,   255),
    'green':  (0,   255, 0),
    'blue':   (255, 0,   0),
    'yellow': (0,   255, 255),
    'orange': (0,   165, 255),
}


# ── 状态机 ────────────────────────────────────────────────────────────────────

class State(Enum):
    IDLE            = 0
    SEARCHING       = 1   # 等待检测到目标
    APPROACHING     = 2   # 移动到目标正上方
    DESCENDING      = 3   # 下降到抓取高度
    GRASPING        = 4   # 关闭夹爪
    LIFTING         = 5   # 提起物体
    MOVING_TO_PLACE = 6   # 移向放置区上方
    PLACING         = 7   # 下降至放置高度
    RELEASING       = 8   # 打开夹爪
    HOMING          = 9   # 归位


# ── 节点 ──────────────────────────────────────────────────────────────────────

class ArmGraspNode(Node):

    def __init__(self):
        super().__init__('arm_grasp_node')

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_area', 50000)
        # 相机内参（若收到 /camera/camera_info 会自动覆盖）
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        # 相机安装位置，相对机械臂基座（米）
        # 假设摄像头固定在基座正上方，俯视工作台
        self.declare_parameter('cam_x', 0.0)    # 水平偏移 X
        self.declare_parameter('cam_y', 0.20)   # 水平偏移 Y（前方）
        self.declare_parameter('cam_z', 0.40)   # 高度
        # 物体在桌面上的高度（米）
        self.declare_parameter('object_height', 0.02)
        # 机械臂连杆长度（米）, 4-DOF: 底座高 + 3段连杆
        self.declare_parameter('link_base', 0.08)   # 底座到肩关节高度
        self.declare_parameter('link1', 0.10)        # 大臂
        self.declare_parameter('link2', 0.10)        # 小臂
        self.declare_parameter('link3', 0.06)        # 腕部到夹爪中心
        # 各关节舵机零点偏移（度）— IK 角度 + 偏移 = 舵机指令
        # 调整此参数让机械臂在 home 位置时所有关节处于 90°（正中）
        self.declare_parameter('joint_offsets', [90, 90, 90, 90])
        # 夹爪：通道号 & 开/合角度（度）
        self.declare_parameter('gripper_channel', 4)
        self.declare_parameter('gripper_open',  90)
        self.declare_parameter('gripper_close', 20)
        # 放置目标位置（基座坐标，米）
        self.declare_parameter('place_x', 0.15)
        self.declare_parameter('place_y', 0.12)
        # 接近/提起高度（米）
        self.declare_parameter('approach_height', 0.08)
        self.declare_parameter('lift_height',     0.10)
        # 每段运动等待时间（秒，视实际舵机速度调整）
        self.declare_parameter('move_duration',  2.0)
        self.declare_parameter('grasp_duration', 1.0)

        # ── 读取参数 ──────────────────────────────────────────────
        p = self.get_parameter
        self.target_color  = p('target_color').value
        self.min_area      = p('min_area').value
        self.max_area      = p('max_area').value
        self.fx            = p('fx').value
        self.fy            = p('fy').value
        self.cx_p          = p('cx').value
        self.cy_p          = p('cy').value
        self.cam_x         = p('cam_x').value
        self.cam_y         = p('cam_y').value
        self.cam_z         = p('cam_z').value
        self.obj_h         = p('object_height').value
        self.Lb            = p('link_base').value
        self.L1            = p('link1').value
        self.L2            = p('link2').value
        self.L3            = p('link3').value
        self.offsets       = list(p('joint_offsets').value)
        self.grip_ch       = p('gripper_channel').value
        self.grip_open     = p('gripper_open').value
        self.grip_close    = p('gripper_close').value
        self.place_x       = p('place_x').value
        self.place_y       = p('place_y').value
        self.approach_h    = p('approach_height').value
        self.lift_h        = p('lift_height').value
        self.move_dur      = p('move_duration').value
        self.grasp_dur     = p('grasp_duration').value

        # ── 运行时状态 ────────────────────────────────────────────
        self.state         = State.IDLE
        self.bridge        = CvBridge()
        self.target_3d: Optional[Tuple[float, float, float]] = None
        self.action_start  = 0.0
        self.cam_calibrated = False

        # ── 订阅 ──────────────────────────────────────────────────
        self.create_subscription(Image,      '/camera/image_raw',   self._img_cb,  10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, 10)
        self.create_subscription(Bool,   '/start_grasp',       self._start_cb, 10)
        self.create_subscription(Bool,   '/stop_grasp',        self._stop_cb,  10)
        self.create_subscription(String, '/set_target_color',  self._color_cb, 10)

        # ── 发布 ──────────────────────────────────────────────────
        self.servo_pub = self.create_publisher(Int32MultiArray, '/servo_commands',  10)
        self.debug_pub = self.create_publisher(Image,           '/arm_debug_image', 10)
        self.state_pub = self.create_publisher(String,          '/arm_state',       10)

        self.create_timer(0.2, self._pub_state)

        # ── 状态机定时器（20 Hz） ─────────────────────────────────
        self.create_timer(0.05, self._fsm)

        self._log_banner()

    # ── 回调 ─────────────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        """自动接收相机内参，覆盖参数默认值"""
        if not self.cam_calibrated:
            self.fx  = msg.k[0]
            self.fy  = msg.k[4]
            self.cx_p = msg.k[2]
            self.cy_p = msg.k[5]
            self.cam_calibrated = True
            self.get_logger().info(
                f'已接收相机内参  fx={self.fx:.1f}  fy={self.fy:.1f}  '
                f'cx={self.cx_p:.1f}  cy={self.cy_p:.1f}')

    def _start_cb(self, msg: Bool):
        if msg.data and self.state == State.IDLE:
            self.state     = State.SEARCHING
            self.target_3d = None
            self.get_logger().info('═══════════════════════════════')
            self.get_logger().info('  视觉抓取任务已启动')
            self.get_logger().info(f'  目标颜色: {self.target_color}')
            self.get_logger().info('═══════════════════════════════')

    def _stop_cb(self, msg: Bool):
        if msg.data:
            self.state        = State.IDLE
            self.target_3d    = None
            self.action_start = 0.0
            self.get_logger().info('任务已手动停止')

    def _color_cb(self, msg: String):
        color = msg.data.lower().strip()
        if color not in COLOR_RANGES:
            self.get_logger().warn(
                f'未知颜色 "{color}"，可选: {list(COLOR_RANGES.keys())}')
            return
        self.target_color = color
        self.get_logger().info(f'目标颜色已切换为: {color}')

    def _pub_state(self):
        msg      = String()
        msg.data = f'{self.state.name}|{self.target_color}'
        self.state_pub.publish(msg)

    def _img_cb(self, msg: Image):
        """图像处理：检测目标，更新 target_3d，发布调试画面"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._build_mask(hsv, self.target_color)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        debug    = frame.copy()
        detected = False

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)

            if self.min_area < area < self.max_area:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    px = int(M['m10'] / M['m00'])
                    py = int(M['m01'] / M['m00'])
                    pos = self._pixel_to_base(px, py)
                    detected = True

                    # 仅在搜索/接近阶段持续更新目标坐标
                    if self.state in (State.SEARCHING, State.APPROACHING):
                        self.target_3d = pos

                    # 绘制检测结果
                    bgr = COLOR_BGR.get(self.target_color, (255, 255, 255))
                    cv2.drawContours(debug, [largest], -1, bgr, 2)
                    cv2.circle(debug, (px, py), 8, bgr, -1)
                    cv2.drawMarker(debug, (px, py), (0, 255, 255),
                                   cv2.MARKER_CROSS, 30, 2)

                    if pos:
                        reachable = self._in_workspace(*pos)
                        ws_str    = 'reachable' if reachable else 'OUT OF RANGE'
                        ws_color  = (0, 255, 0) if reachable else (0, 60, 255)
                        info = (f'{self.target_color}  '
                                f'x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}m'
                                f'  [{ws_str}]')
                        cv2.putText(debug, info, (px - 80, py - 16),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, ws_color, 2)
                    else:
                        cv2.putText(debug, f'{self.target_color}  (workspace unknown)',
                                    (px - 60, py - 16),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)

        if not detected and self.state == State.SEARCHING:
            self.target_3d = None

        # HUD 覆盖层
        self._draw_hud(debug, detected)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ── 检测辅助 ──────────────────────────────────────────────────────────────

    def _build_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        ranges = COLOR_RANGES.get(color, [])
        mask   = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        return mask

    def _pixel_to_base(self, px: int, py: int) -> Optional[Tuple[float, float, float]]:
        """
        俯视相机模型：将像素坐标投影到已知高度的桌面平面。
        假设相机光轴垂直向下，cam_y 轴对应机械臂基座 X 轴正方向。
        """
        depth = self.cam_z - self.obj_h          # 相机到桌面的距离
        if depth <= 0:
            return None
        cam_x_m = (px - self.cx_p) * depth / self.fx
        cam_y_m = (py - self.cy_p) * depth / self.fy
        # 坐标轴映射（根据实际相机安装方向调整符号）
        base_x = self.cam_y - cam_y_m
        base_y = self.cam_x - cam_x_m
        base_z = self.obj_h
        return (base_x, base_y, base_z)

    def _in_workspace(self, x: float, y: float, z: float) -> bool:
        r     = math.sqrt(x ** 2 + y ** 2)
        reach = self.L1 + self.L2 + self.L3
        return 0.03 < r < reach * 0.95 and abs(y) < 0.25 and x > 0

    # ── 逆运动学 ──────────────────────────────────────────────────────────────

    def _ik(self, x: float, y: float, z: float) -> Optional[List[int]]:
        """
        4-DOF 解析逆运动学，末端执行器竖直向下。

        关节定义（与 PCA9685 通道对应）:
          ch0  θ1  底座旋转（绕 Z 轴）
          ch1  θ2  肩关节
          ch2  θ3  肘关节
          ch3  θ4  腕关节（保持末端垂直向下）

        返回 None 表示目标不可达。
        """
        # 底座旋转角
        theta1 = math.atan2(y, x)

        # 在 (r, z) 平面做 2-link IK
        r  = math.sqrt(x ** 2 + y ** 2)
        # 腕关节位置（扣除腕部连杆后的目标点）
        wr = r
        wz = z - self.Lb + self.L3   # 换算到肩关节坐标系

        d = math.sqrt(wr ** 2 + wz ** 2)
        if d < 1e-4 or d > (self.L1 + self.L2):
            self.get_logger().warn(
                f'IK: 目标不可达  d={d:.3f} m  最大={self.L1+self.L2:.3f} m  '
                f'pos=({x:.3f},{y:.3f},{z:.3f})')
            return None

        # 余弦定理求肘关节角
        cos3 = (self.L1 ** 2 + self.L2 ** 2 - d ** 2) / (2 * self.L1 * self.L2)
        cos3 = float(np.clip(cos3, -1.0, 1.0))
        theta3 = math.acos(cos3) - math.pi  # 肘部向下构型

        # 肩关节角
        alpha    = math.atan2(wz, wr)
        beta_cos = float(np.clip(
            (self.L1 ** 2 + d ** 2 - self.L2 ** 2) / (2 * self.L1 * d), -1.0, 1.0))
        beta     = math.acos(beta_cos)
        theta2   = alpha + beta

        # 腕关节角：保持末端垂直向下
        theta4 = -(theta2 + theta3) - math.pi / 2

        angles_rad = [theta1, theta2, theta3, theta4]

        servo_angles: List[int] = []
        for rad, offset in zip(angles_rad, self.offsets):
            deg = int(np.clip(math.degrees(rad) + offset, 0, 180))
            servo_angles.append(deg)

        self.get_logger().debug(
            f'IK result: {[f"{a}°" for a in servo_angles]}')
        return servo_angles

    # ── 舵机指令 ──────────────────────────────────────────────────────────────

    def _send(self, joints: List[int], gripper: int):
        """发布舵机指令：通道 0–3 为关节角度，通道 grip_ch 为夹爪"""
        data = list(joints)
        # 确保数组长度覆盖夹爪通道
        while len(data) <= self.grip_ch:
            data.append(90)
        data[self.grip_ch] = gripper
        msg      = Int32MultiArray()
        msg.data = data
        self.servo_pub.publish(msg)

    def _home_joints(self) -> List[int]:
        return [o for o in self.offsets]

    def _send_home(self):
        self._send(self._home_joints(), self.grip_open)

    # ── 计时辅助 ──────────────────────────────────────────────────────────────

    def _wait(self, duration: float) -> bool:
        if self.action_start == 0.0:
            self.action_start = time.time()
            return False
        done = (time.time() - self.action_start) >= duration
        if done:
            self.action_start = 0.0
        return done

    # ── 状态机 ────────────────────────────────────────────────────────────────

    def _fsm(self):
        s = self.state

        if s == State.IDLE:
            return

        elif s == State.SEARCHING:
            if self.target_3d and self._in_workspace(*self.target_3d):
                self.get_logger().info(
                    f'[SEARCH] 检测到目标  '
                    f'({self.target_3d[0]:.3f}, {self.target_3d[1]:.3f}, '
                    f'{self.target_3d[2]:.3f}) m')
                self._send(self._home_joints(), self.grip_open)
                self.state = State.APPROACHING

        elif s == State.APPROACHING:
            if not self.target_3d:
                self.state = State.SEARCHING
                return
            tx, ty, tz = self.target_3d
            joints = self._ik(tx, ty, tz + self.approach_h)
            if joints:
                self._send(joints, self.grip_open)
                self.get_logger().info('[APPROACH] 移动到目标上方')
                if self._wait(self.move_dur):
                    self.state = State.DESCENDING
            else:
                self.get_logger().error('[APPROACH] IK 失败，重新搜索')
                self.state = State.SEARCHING

        elif s == State.DESCENDING:
            joints = self._ik(*self.target_3d)
            if joints:
                self._send(joints, self.grip_open)
                self.get_logger().info('[DESCEND] 下降到抓取位置')
                if self._wait(1.5):
                    self.state = State.GRASPING
            else:
                self.get_logger().error('[DESCEND] IK 失败，重新搜索')
                self.state = State.SEARCHING

        elif s == State.GRASPING:
            joints = self._ik(*self.target_3d)
            if joints:
                self._send(joints, self.grip_close)
            self.get_logger().info('[GRASP] 关闭夹爪')
            if self._wait(self.grasp_dur):
                self.state = State.LIFTING

        elif s == State.LIFTING:
            tx, ty, tz = self.target_3d
            joints = self._ik(tx, ty, tz + self.lift_h)
            if joints:
                self._send(joints, self.grip_close)
                self.get_logger().info('[LIFT] 提起物体')
                if self._wait(1.5):
                    self.state = State.MOVING_TO_PLACE

        elif s == State.MOVING_TO_PLACE:
            joints = self._ik(self.place_x, self.place_y,
                              self.obj_h + self.lift_h)
            if joints:
                self._send(joints, self.grip_close)
                self.get_logger().info('[MOVE] 移向放置区')
                if self._wait(self.move_dur):
                    self.state = State.PLACING

        elif s == State.PLACING:
            joints = self._ik(self.place_x, self.place_y, self.obj_h)
            if joints:
                self._send(joints, self.grip_close)
                self.get_logger().info('[PLACE] 下降放置')
                if self._wait(1.5):
                    self.state = State.RELEASING

        elif s == State.RELEASING:
            joints = self._ik(self.place_x, self.place_y, self.obj_h)
            if joints:
                self._send(joints, self.grip_open)
            self.get_logger().info('[RELEASE] 打开夹爪')
            if self._wait(0.8):
                self.state = State.HOMING

        elif s == State.HOMING:
            self._send_home()
            self.get_logger().info('[HOME] 归位中...')
            if self._wait(self.move_dur):
                self.target_3d = None
                self.state     = State.IDLE
                self.get_logger().info('═══════════════════════════════')
                self.get_logger().info('  抓取任务完成！')
                self.get_logger().info('═══════════════════════════════')

    # ── 调试 HUD ──────────────────────────────────────────────────────────────

    def _draw_hud(self, img: np.ndarray, detected: bool):
        h, w = img.shape[:2]

        # 半透明背景条
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 70), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, img, 0.55, 0, img)

        state_color = {
            State.IDLE:            (120, 120, 120),
            State.SEARCHING:       (0, 200, 255),
            State.APPROACHING:     (0, 255, 100),
            State.DESCENDING:      (0, 255, 100),
            State.GRASPING:        (0, 255, 0),
            State.LIFTING:         (0, 255, 0),
            State.MOVING_TO_PLACE: (255, 200, 0),
            State.PLACING:         (255, 200, 0),
            State.RELEASING:       (100, 100, 255),
            State.HOMING:          (180, 180, 180),
        }.get(self.state, (255, 255, 255))

        cv2.putText(img, f'State : {self.state.name}', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
        cv2.putText(img, f'Target: {self.target_color}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        status = 'DETECTED' if detected else 'searching...'
        s_color = (0, 255, 80) if detected else (0, 80, 255)
        cv2.putText(img, status, (w - 160, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, s_color, 2)

    def _log_banner(self):
        self.get_logger().info('━' * 48)
        self.get_logger().info('  arm_grasp_node 已启动')
        self.get_logger().info(f'  检测颜色 : {self.target_color}')
        self.get_logger().info(f'  调试图像 : /arm_debug_image')
        self.get_logger().info('  控制:')
        self.get_logger().info('    ros2 topic pub /start_grasp std_msgs/msg/Bool'
                               ' "data: true" --once')
        self.get_logger().info('    ros2 topic pub /stop_grasp  std_msgs/msg/Bool'
                               ' "data: true" --once')
        self.get_logger().info('━' * 48)


# ── 入口 ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ArmGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
