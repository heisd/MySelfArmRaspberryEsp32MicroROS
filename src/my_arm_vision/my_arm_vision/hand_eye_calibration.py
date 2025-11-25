#!/usr/bin/env python3
"""
手眼标定节点
Eye-to-Hand 配置（固定摄像头观察机械臂工作空间）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from pathlib import Path
from typing import List, Tuple, Optional


class HandEyeCalibration(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration')
        
        # 参数
        self.declare_parameter('checkerboard_size', [6, 9])  # 内角点数
        self.declare_parameter('square_size', 0.025)         # 方格大小（米）
        self.declare_parameter('calibration_file', 
            '/home/pi/arm_ws/config/hand_eye_calibration.yaml')
        
        board_size = self.get_parameter('checkerboard_size').value
        self.board_size = (board_size[0], board_size[1])
        self.square_size = self.get_parameter('square_size').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        self.bridge = CvBridge()
        
        # 标定数据存储
        self.robot_poses: List[np.ndarray] = []  # 机械臂末端位姿
        self.target_poses: List[np.ndarray] = []  # 标定板位姿（相机坐标系）
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 棋盘格3D点
        self.objp = np.zeros((self.board_size[0] * self.board_size[1], 3), 
                             np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_size[0], 
                                    0:self.board_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # 发布
        self.debug_pub = self.create_publisher(
            Image, '/calibration_debug_image', 10)
        
        # 当前图像
        self.current_image: Optional[np.ndarray] = None
        
        self.get_logger().info('手眼标定节点已启动')
        self.get_logger().info(f'棋盘格大小: {self.board_size}')
        self.get_logger().info('按键说明:')
        self.get_logger().info('  c - 采集当前位姿')
        self.get_logger().info('  s - 开始标定')
        self.get_logger().info('  l - 加载已有标定')
        
        # 键盘输入定时器
        self.create_timer(0.1, self.keyboard_callback)

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def detect_checkerboard(self, image: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """检测棋盘格"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 
                       30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                       criteria)
        return ret, corners

    def get_robot_pose(self) -> Optional[np.ndarray]:
        """获取机械臂末端位姿"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'gripper_base',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # 转换为4x4齐次矩阵
            t = trans.transform.translation
            r = trans.transform.rotation
            
            from scipy.spatial.transform import Rotation
            rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
            
            pose = np.eye(4)
            pose[:3, :3] = rot.as_matrix()
            pose[:3, 3] = [t.x, t.y, t.z]
            
            return pose
        except Exception as e:
            self.get_logger().error(f'获取机械臂位姿失败: {e}')
            return None

    def capture_pose(self):
        """采集一组位姿"""
        if self.current_image is None:
            self.get_logger().warn('没有图像')
            return
        
        if self.camera_matrix is None:
            self.get_logger().warn('没有相机内参')
            return
        
        # 检测棋盘格
        ret, corners = self.detect_checkerboard(self.current_image)
        if not ret:
            self.get_logger().warn('未检测到棋盘格')
            return
        
        # 计算标定板位姿（相机坐标系）
        ret, rvec, tvec = cv2.solvePnP(
            self.objp, corners, self.camera_matrix, self.dist_coeffs)
        
        if not ret:
            self.get_logger().warn('无法计算标定板位姿')
            return
        
        # 获取机械臂位姿
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
        
        # 转换为4x4矩阵
        R, _ = cv2.Rodrigues(rvec)
        target_pose = np.eye(4)
        target_pose[:3, :3] = R
        target_pose[:3, 3] = tvec.flatten()
        
        self.robot_poses.append(robot_pose)
        self.target_poses.append(target_pose)
        
        self.get_logger().info(
            f'已采集 {len(self.robot_poses)} 组位姿')
        
        # 显示结果
        vis_image = self.current_image.copy()
        cv2.drawChessboardCorners(vis_image, self.board_size, corners, ret)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, 'bgr8'))

    def calibrate(self):
        """执行手眼标定"""
        if len(self.robot_poses) < 3:
            self.get_logger().error('需要至少3组位姿')
            return
        
        self.get_logger().info(f'开始标定，共 {len(self.robot_poses)} 组数据')
        
        # 准备数据
        R_gripper2base = [p[:3, :3] for p in self.robot_poses]
        t_gripper2base = [p[:3, 3] for p in self.robot_poses]
        R_target2cam = [p[:3, :3] for p in self.target_poses]
        t_target2cam = [p[:3, 3] for p in self.target_poses]
        
        # Eye-to-Hand 标定
        # 求解: base_T_camera
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        # 创建变换矩阵
        transform = np.eye(4)
        transform[:3, :3] = R_cam2base
        transform[:3, 3] = t_cam2base.flatten()
        
        self.get_logger().info('标定完成!')
        self.get_logger().info(f'相机到基座变换:\n{transform}')
        
        # 保存标定结果
        self.save_calibration(transform)
        
        # 发布TF
        self.publish_transform(transform)

    def save_calibration(self, transform: np.ndarray):
        """保存标定结果"""
        data = {
            'camera_to_base': transform.tolist()
        }
        
        Path(self.calibration_file).parent.mkdir(parents=True, exist_ok=True)
        with open(self.calibration_file, 'w') as f:
            yaml.dump(data, f)
        
        self.get_logger().info(f'标定结果已保存: {self.calibration_file}')

    def load_calibration(self):
        """加载标定结果"""
        try:
            with open(self.calibration_file, 'r') as f:
                data = yaml.safe_load(f)
            
            transform = np.array(data['camera_to_base'])
            self.publish_transform(transform)
            self.get_logger().info('标定结果已加载')
        except Exception as e:
            self.get_logger().error(f'加载失败: {e}')

    def publish_transform(self, transform: np.ndarray):
        """发布相机到基座的TF"""
        from scipy.spatial.transform import Rotation
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        t.transform.translation.x = transform[0, 3]
        t.transform.translation.y = transform[1, 3]
        t.transform.translation.z = transform[2, 3]
        
        rot = Rotation.from_matrix(transform[:3, :3])
        q = rot.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def keyboard_callback(self):
        """处理键盘输入（简化版，实际可用其他方式）"""
        # 这里可以通过服务或话题来触发
        pass


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()