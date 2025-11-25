#!/usr/bin/env python3
"""
简化版视觉抓取节点
整合检测、坐标转换、运动控制
适合快速测试和学习
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum


class State(Enum):
    IDLE = 0
    SEARCHING = 1
    APPROACHING = 2
    DESCENDING = 3
    GRASPING = 4
    LIFTING = 5
    MOVING = 6
    PLACING = 7
    RELEASING = 8
    HOMING = 9


@dataclass
class CameraConfig:
    """相机配置"""
    # 相机内参（需要根据你的相机标定）
    fx: float = 600.0
    fy: float = 600.0
    cx: float = 320.0
    cy: float = 240.0
    
    # 相机位置（相对于机械臂基座，需要手眼标定）
    # 假设相机在基座正前方上方，朝下看
    cam_x: float = 0.0      # 相机X位置
    cam_y: float = 0.20     # 相机Y位置（前方20cm）
    cam_z: float = 0.40     # 相机Z位置（上方40cm）
    
    # 相机朝向（俯视角度，弧度）
    tilt_angle: float = -1.57  # 朝下看（-90度）


@dataclass 
class ArmConfig:
    """机械臂配置"""
    num_joints: int = 4
    home_position: List[float] = None
    
    # 工作空间限制
    x_min: float = 0.05
    x_max: float = 0.25
    y_min: float = -0.15
    y_max: float = 0.15
    z_min: float = 0.02
    z_max: float = 0.20
    
    def __post_init__(self):
        if self.home_position is None:
            self.home_position = [0.0, 0.0, 0.0, 0.0]


class SimpleVisualGrasp(Node):
    def __init__(self):
        super().__init__('simple_visual_grasp')
        
        # 配置
        self.camera = CameraConfig()
        self.arm = ArmConfig()
        
        # 状态
        self.state = State.IDLE
        self.bridge = CvBridge()
        
        # 目标信息
        self.target_pixel: Optional[Tuple[int, int]] = None
        self.target_3d: Optional[Tuple[float, float, float]] = None
        self.object_detected = False
        
        # 放置位置（基座坐标系）
        self.place_position = (0.15, 0.12, 0.05)
        
        # 当前关节位置
        self.current_joints = [0.0] * self.arm.num_joints
        
        # HSV颜色范围（检测红色物体）
        self.hsv_lower = np.array([0, 100, 100])
        self.hsv_upper = np.array([10, 255, 255])
        self.hsv_lower2 = np.array([160, 100, 100])
        self.hsv_upper2 = np.array([180, 255, 255])
        
        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states_feedback', self.joint_state_callback, 10)
        
        # 发布
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)
        self.gripper_pub = self.create_publisher(
            Bool, '/gripper_command', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/grasp_debug_image', 10)
        
        # 主循环定时器
        self.create_timer(0.05, self.main_loop)  # 20Hz
        
        # 动作执行定时器
        self.action_start_time = 0.0
        self.action_duration = 0.0
        
        self.get_logger().info('='*50)
        self.get_logger().info('简化版视觉抓取节点已启动')
        self.get_logger().info('='*50)
        self.get_logger().info('命令:')
        self.get_logger().info('  发布 Bool(True) 到 /start_grasp 开始抓取')
        self.get_logger().info('  发布 Bool(True) 到 /stop_grasp 停止')
        
        # 控制订阅
        self.start_sub = self.create_subscription(
            Bool, '/start_grasp', self.start_callback, 10)
        self.stop_sub = self.create_subscription(
            Bool, '/stop_grasp', self.stop_callback, 10)

    def start_callback(self, msg: Bool):
        if msg.data and self.state == State.IDLE:
            self.state = State.SEARCHING
            self.get_logger().info('开始视觉抓取任务')

    def stop_callback(self, msg: Bool):
        if msg.data:
            self.state = State.IDLE
            self.get_logger().info('任务已停止')

    def joint_state_callback(self, msg: JointState):
        """接收关节状态反馈"""
        for i, pos in enumerate(msg.position):
            if i < len(self.current_joints):
                self.current_joints[i] = pos

    def image_callback(self, msg: Image):
        """处理图像，检测物体"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return
        
        # 转HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 创建颜色掩膜（红色需要两个范围）
        mask1 = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = mask1 | mask2
        
        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        # 创建调试图像
        debug_image = cv_image.copy()
        
        # 找最大轮廓
        self.object_detected = False
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if 500 < area < 50000:  # 面积过滤
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    self.target_pixel = (cx, cy)
                    self.object_detected = True
                    
                    # 转换到3D坐标
                    self.target_3d = self.pixel_to_base_frame(cx, cy)
                    
                    # 绘制检测结果
                    cv2.drawContours(debug_image, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(debug_image, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.drawMarker(debug_image, (cx, cy), (255, 255, 0),
                                  cv2.MARKER_CROSS, 20, 2)
                    
                    # 显示坐标
                    if self.target_3d:
                        text = f'({self.target_3d[0]:.3f}, {self.target_3d[1]:.3f}, {self.target_3d[2]:.3f})'
                        cv2.putText(debug_image, text, (cx - 60, cy - 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # 显示状态
        cv2.putText(debug_image, f'State: {self.state.name}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 发布调试图像
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))

    def pixel_to_base_frame(self, px: int, py: int) -> Optional[Tuple[float, float, float]]:
        """
        将像素坐标转换到机械臂基座坐标系
        假设：
        - 相机在基座上方，朝下看
        - 物体在桌面上（已知高度）
        """
        # 假设物体高度（桌面）
        object_z = 0.02  # 2cm
        
        # 计算深度（相机到物体的距离）
        depth = self.camera.cam_z - object_z
        
        # 像素坐标转相机坐标
        cam_x = (px - self.camera.cx) * depth / self.camera.fx
        cam_y = (py - self.camera.cy) * depth / self.camera.fy
        cam_z = depth
        
        # 相机坐标转基座坐标
        # 假设相机朝下安装，需要根据实际安装调整
        # 这里假设相机光轴与Z轴平行，朝下
        base_x = self.camera.cam_y - cam_y  # 相机Y对应基座X
        base_y = -cam_x                      # 相机X对应基座-Y
        base_z = object_z
        
        # 检查是否在工作空间内
        if (self.arm.x_min <= base_x <= self.arm.x_max and
            self.arm.y_min <= base_y <= self.arm.y_max):
            return (base_x, base_y, base_z)
        
        return None

    def send_joint_command(self, positions: List[float]):
        """发送关节命令"""
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_cmd_pub.publish(msg)

    def control_gripper(self, close: bool):
        """控制夹爪"""
        msg = Bool()
        msg.data = close
        self.gripper_pub.publish(msg)

    def inverse_kinematics(self, x: float, y: float, z: float) -> Optional[List[float]]:
        """
        简化的逆运动学
        假设4自由度机械臂：
        - joint1: 底座旋转
        - joint2: 肩部
        - joint3: 肘部  
        - joint4: 腕部
        
        你需要根据实际机械臂调整！
        """
        # 机械臂参数（根据你的URDF调整）
        L1 = 0.10  # link1 长度
        L2 = 0.10  # link2 长度
        L3 = 0.08  # link3 长度
        L4 = 0.05  # link4 到末端长度
        
        # 计算基座旋转角
        theta1 = np.arctan2(y, x)
        
        # 计算水平距离
        r = np.sqrt(x**2 + y**2)
        
        # 目标点（考虑末端长度）
        # 假设末端朝下
        target_r = r
        target_z = z + L4
        
        # 2连杆逆运动学
        d = np.sqrt(target_r**2 + (target_z - L1)**2)
        
        # 检查可达性
        if d > L2 + L3 or d < abs(L2 - L3):
            self.get_logger().warn(f'目标不可达: d={d:.3f}')
            return None
        
        # 肘部角度（使用余弦定理）
        cos_theta3 = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
        cos_theta3 = np.clip(cos_theta3, -1, 1)
        theta3 = np.arccos(cos_theta3) - np.pi  # 肘部向下
        
        # 肩部角度
        alpha = np.arctan2(target_z - L1, target_r)
        beta = np.arccos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
        theta2 = alpha + beta
        
        # 腕部角度（保持末端朝下）
        theta4 = -theta2 - theta3 - np.pi/2
        
        # 角度限制
        joints = [theta1, theta2, theta3, theta4]
        limits = [
            (-1.57, 1.57),
            (-1.57, 1.57),
            (-1.57, 1.57),
            (-1.57, 1.57)
        ]
        
        for i, (angle, (lo, hi)) in enumerate(zip(joints, limits)):
            if angle < lo or angle > hi:
                self.get_logger().warn(f'关节{i+1}超限: {angle:.3f}')
                joints[i] = np.clip(angle, lo, hi)
        
        return joints

    def wait_action(self, duration: float) -> bool:
        """检查动作是否完成"""
        if self.action_start_time == 0:
            self.action_start_time = time.time()
            return False
        
        if time.time() - self.action_start_time >= duration:
            self.action_start_time = 0
            return True
        return False

    def main_loop(self):
        """主状态机循环"""
        
        if self.state == State.IDLE:
            pass
        
        elif self.state == State.SEARCHING:
            # 等待检测到物体
            if self.object_detected and self.target_3d:
                self.get_logger().info(
                    f'检测到目标: ({self.target_3d[0]:.3f}, '
                    f'{self.target_3d[1]:.3f}, {self.target_3d[2]:.3f})')
                
                # 打开夹爪
                self.control_gripper(False)
                self.state = State.APPROACHING
        
        elif self.state == State.APPROACHING:
            # 移动到目标上方
            if self.target_3d:
                approach_z = self.target_3d[2] + 0.08  # 上方8cm
                joints = self.inverse_kinematics(
                    self.target_3d[0], self.target_3d[1], approach_z)
                
                if joints:
                    self.send_joint_command(joints)
                    self.get_logger().info('移动到接近位置')
                    
                    if self.wait_action(2.0):
                        self.state = State.DESCENDING
                else:
                    self.get_logger().error('无法计算接近位置')
                    self.state = State.IDLE
        
        elif self.state == State.DESCENDING:
            # 下降到抓取位置
            if self.target_3d:
                grasp_z = self.target_3d[2] + 0.01  # 略高于物体
                joints = self.inverse_kinematics(
                    self.target_3d[0], self.target_3d[1], grasp_z)
                
                if joints:
                    self.send_joint_command(joints)
                    self.get_logger().info('下降到抓取位置')
                    
                    if self.wait_action(1.5):
                        self.state = State.GRASPING
        
        elif self.state == State.GRASPING:
            # 关闭夹爪
            self.control_gripper(True)
            self.get_logger().info('抓取物体')
            
            if self.wait_action(1.0):
                self.state = State.LIFTING
        
        elif self.state == State.LIFTING:
            # 提起物体
            if self.target_3d:
                lift_z = self.target_3d[2] + 0.10
                joints = self.inverse_kinematics(
                    self.target_3d[0], self.target_3d[1], lift_z)
                
                if joints:
                    self.send_joint_command(joints)
                    self.get_logger().info('提起物体')
                    
                    if self.wait_action(1.5):
                        self.state = State.MOVING
        
        elif self.state == State.MOVING:
            # 移动到放置位置上方
            place_z = self.place_position[2] + 0.10
            joints = self.inverse_kinematics(
                self.place_position[0], self.place_position[1], place_z)
            
            if joints:
                self.send_joint_command(joints)
                self.get_logger().info('移动到放置位置')
                
                if self.wait_action(2.0):
                    self.state = State.PLACING
        
        elif self.state == State.PLACING:
            # 下降到放置位置
            joints = self.inverse_kinematics(
                self.place_position[0], 
                self.place_position[1], 
                self.place_position[2])
            
            if joints:
                self.send_joint_command(joints)
                self.get_logger().info('放置物体')
                
                if self.wait_action(1.5):
                    self.state = State.RELEASING
        
        elif self.state == State.RELEASING:
            # 打开夹爪
            self.control_gripper(False)
            self.get_logger().info('释放物体')
            
            if self.wait_action(0.5):
                self.state = State.HOMING
        
        elif self.state == State.HOMING:
            # 返回初始位置
            self.send_joint_command(self.arm.home_position)
            self.get_logger().info('返回初始位置')
            
            if self.wait_action(2.0):
                self.get_logger().info('='*50)
                self.get_logger().info('抓取任务完成!')
                self.get_logger().info('='*50)
                self.state = State.IDLE
                self.target_3d = None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVisualGrasp()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()