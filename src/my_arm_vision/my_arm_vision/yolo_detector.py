#!/usr/bin/env python3
"""
YOLO 物体检测节点
使用 YOLOv8 进行通用物体检测
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Optional

# 需要安装: pip install ultralytics --break-system-packages
from ultralytics import YOLO


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 参数
        self.declare_parameter('model_path', 'yolov8n.pt')  # nano 模型适合树莓派
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_classes', ['cup', 'bottle', 'apple', 'orange'])
        self.declare_parameter('camera_topic', '/camera/image_raw')
        
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_classes = self.get_parameter('target_classes').value
        camera_topic = self.get_parameter('camera_topic').value
        
        # 加载 YOLO 模型
        self.get_logger().info(f'正在加载模型: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('模型加载完成')
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # 订阅
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # 发布
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/yolo_detections', 10)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/target_object_pose', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/detection_debug_image', 10)

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def pixel_to_3d(self, px: int, py: int, depth: float = 0.3):
        """像素转3D坐标"""
        if self.camera_matrix is not None:
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        else:
            fx, fy, cx, cy = 600, 600, 320, 240
        
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        return x, y, depth

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return
        
        # YOLO 推理
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_link'
        
        best_detection = None
        best_conf = 0
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                
                # 只处理目标类别
                if cls_name not in self.target_classes:
                    continue
                
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                
                # 绘制检测框
                cv2.rectangle(cv_image, (int(x1), int(y1)), 
                             (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f'{cls_name}: {conf:.2f}',
                           (int(x1), int(y1) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # 创建检测消息
                det = Detection2D()
                det.bbox.center.position.x = float(cx)
                det.bbox.center.position.y = float(cy)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                detection_array.detections.append(det)
                
                # 选择置信度最高的作为目标
                if conf > best_conf:
                    best_conf = conf
                    best_detection = (cx, cy, cls_name)
        
        # 发布目标位姿
        if best_detection:
            cx, cy, cls_name = best_detection
            x, y, z = self.pixel_to_3d(cx, cy)
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'camera_link'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            self.target_pose_pub.publish(pose)
            
            cv2.drawMarker(cv_image, (cx, cy), (0, 0, 255), 
                          cv2.MARKER_CROSS, 30, 3)
        
        self.detection_pub.publish(detection_array)
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()