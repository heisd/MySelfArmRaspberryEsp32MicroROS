#!/usr/bin/env python3
"""
物体检测节点 - 基于颜色检测
支持检测特定颜色的物体并发布其位置
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class DetectedObject:
    """检测到的物体"""
    center_pixel: Tuple[int, int]  # 像素坐标
    contour: np.ndarray            # 轮廓
    area: float                    # 面积
    color_name: str                # 颜色名称


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # 参数声明
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('min_object_area', 500)
        self.declare_parameter('max_object_area', 50000)
        self.declare_parameter('detection_colors', ['red', 'green', 'blue'])
        
        # 获取参数
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.min_area = self.get_parameter('min_object_area').value
        self.max_area = self.get_parameter('max_object_area').value
        self.target_colors = self.get_parameter('detection_colors').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # HSV 颜色范围定义
        self.color_ranges = {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'green': [
                (np.array([35, 100, 100]), np.array([85, 255, 255]))
            ],
            'blue': [
                (np.array([100, 100, 100]), np.array([130, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([35, 255, 255]))
            ],
            'orange': [
                (np.array([10, 100, 100]), np.array([20, 255, 255]))
            ]
        }
        
        # 订阅者
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        # 发布者
        self.detection_pub = self.create_publisher(
            PoseArray, '/detected_objects', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/detection_debug_image', 10)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/target_object_pose', 10)
        
        self.get_logger().info('物体检测节点已启动')
        self.get_logger().info(f'目标颜色: {self.target_colors}')

    def camera_info_callback(self, msg: CameraInfo):
        """接收相机内参"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('已接收相机内参')

    def detect_color_objects(self, hsv_image: np.ndarray, 
                             color_name: str) -> List[DetectedObject]:
        """检测特定颜色的物体"""
        if color_name not in self.color_ranges:
            return []
        
        # 创建颜色掩膜
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lower, upper in self.color_ranges[color_name]:
            mask |= cv2.inRange(hsv_image, lower, upper)
        
        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    detected.append(DetectedObject(
                        center_pixel=(cx, cy),
                        contour=contour,
                        area=area,
                        color_name=color_name
                    ))
        
        return detected

    def pixel_to_camera_frame(self, pixel_x: int, pixel_y: int, 
                              depth: float = 0.3) -> Tuple[float, float, float]:
        """
        将像素坐标转换为相机坐标系下的3D坐标
        假设已知深度或使用固定深度估计
        """
        if self.camera_matrix is None:
            # 使用默认相机参数
            fx, fy = 600.0, 600.0
            cx, cy = 320.0, 240.0
        else:
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
        
        # 反投影到相机坐标系
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth
        
        return (x, y, z)

    def image_callback(self, msg: Image):
        """处理图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return
        
        # 转换到 HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 检测所有目标颜色
        all_objects: List[DetectedObject] = []
        for color in self.target_colors:
            objects = self.detect_color_objects(hsv, color)
            all_objects.extend(objects)
        
        # 创建调试图像
        debug_image = cv_image.copy()
        
        # 发布检测结果
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera_link'
        
        # 选择最大的物体作为目标
        target_object: Optional[DetectedObject] = None
        
        for obj in all_objects:
            # 绘制轮廓
            color_bgr = {
                'red': (0, 0, 255),
                'green': (0, 255, 0),
                'blue': (255, 0, 0),
                'yellow': (0, 255, 255),
                'orange': (0, 165, 255)
            }.get(obj.color_name, (255, 255, 255))
            
            cv2.drawContours(debug_image, [obj.contour], -1, color_bgr, 2)
            cv2.circle(debug_image, obj.center_pixel, 5, color_bgr, -1)
            
            # 显示信息
            text = f'{obj.color_name}: {int(obj.area)}'
            cv2.putText(debug_image, text, 
                       (obj.center_pixel[0] - 30, obj.center_pixel[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)
            
            # 转换为3D坐标
            x, y, z = self.pixel_to_camera_frame(
                obj.center_pixel[0], obj.center_pixel[1])
            
            pose = PoseStamped()
            pose.header.frame_id = 'camera_link'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            pose_array.poses.append(pose.pose)
            
            # 选择最大物体
            if target_object is None or obj.area > target_object.area:
                target_object = obj
        
        # 发布最大物体为目标
        if target_object:
            x, y, z = self.pixel_to_camera_frame(
                target_object.center_pixel[0], 
                target_object.center_pixel[1])
            
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'camera_link'
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            target_pose.pose.orientation.w = 1.0
            self.target_pose_pub.publish(target_pose)
            
            # 在目标上画十字
            cv2.drawMarker(debug_image, target_object.center_pixel,
                          (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        
        # 发布结果
        self.detection_pub.publish(pose_array)
        
        # 发布调试图像
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()