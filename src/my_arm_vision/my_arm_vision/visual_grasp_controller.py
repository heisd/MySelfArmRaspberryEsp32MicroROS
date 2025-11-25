#!/usr/bin/env python3
"""
视觉抓取控制器
整合物体检测和机械臂控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, 
    PositionConstraint, OrientationConstraint,
    BoundingVolume, SolidPrimitive
)
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
from enum import Enum
from typing import Optional


class GraspState(Enum):
    IDLE = 0
    DETECTING = 1
    APPROACHING = 2
    GRASPING = 3
    LIFTING = 4
    MOVING = 5
    PLACING = 6
    RELEASING = 7
    RETURNING = 8


class VisualGraspController(Node):
    def __init__(self):
        super().__init__('visual_grasp_controller')
        
        # 参数
        self.declare_parameter('approach_distance', 0.08)
        self.declare_parameter('grasp_height_offset', 0.02)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('place_position', [0.15, 0.15, 0.05])
        
        self.approach_dist = self.get_parameter('approach_distance').value
        self.grasp_offset = self.get_parameter('grasp_height_offset').value
        self.lift_height = self.get_parameter('lift_height').value
        self.place_pos = self.get_parameter('place_position').value
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 状态
        self.state = GraspState.IDLE
        self.target_pose: Optional[PoseStamped] = None
        
        # 订阅目标物体位姿
        self.target_sub = self.create_subscription(
            PoseStamped, '/target_object_pose', 
            self.target_callback, 10)
        
        # 发布夹爪控制
        self.gripper_pub = self.create_publisher(
            Bool, '/gripper_command', 10)
        
        # MoveIt Action Client
        self.move_group_client = ActionClient(
            self, MoveGroup, 'move_action')
        
        # 服务
        self.grasp_srv = self.create_service(
            Trigger, 'start_grasp', self.start_grasp_callback)
        self.stop_srv = self.create_service(
            Trigger, 'stop_grasp', self.stop_grasp_callback)
        
        # 状态机定时器
        self.create_timer(0.1, self.state_machine_callback)
        
        self.get_logger().info('视觉抓取控制器已启动')
        self.get_logger().info('调用 /start_grasp 服务开始抓取')

    def target_callback(self, msg: PoseStamped):
        """接收目标物体位姿"""
        if self.state == GraspState.DETECTING:
            # 转换到基座坐标系
            try:
                transformed = self.tf_buffer.transform(
                    msg, 'base_link', 
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self.target_pose = transformed
                self.get_logger().info(
                    f'目标位置: ({transformed.pose.position.x:.3f}, '
                    f'{transformed.pose.position.y:.3f}, '
                    f'{transformed.pose.position.z:.3f})')
            except Exception as e:
                self.get_logger().warn(f'坐标转换失败: {e}')

    def start_grasp_callback(self, request, response):
        """启动抓取"""
        if self.state == GraspState.IDLE:
            self.state = GraspState.DETECTING
            response.success = True
            response.message = '开始检测目标'
        else:
            response.success = False
            response.message = f'当前状态: {self.state.name}'
        return response

    def stop_grasp_callback(self, request, response):
        """停止抓取"""
        self.state = GraspState.IDLE
        self.target_pose = None
        response.success = True
        response.message = '已停止'
        return response

    def control_gripper(self, close: bool):
        """控制夹爪"""
        msg = Bool()
        msg.data = close
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'夹爪: {"关闭" if close else "打开"}')

    async def move_to_pose(self, pose: Pose) -> bool:
        """移动到指定位姿"""
        # 等待 action server
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server 不可用')
            return False
        
        # 创建运动规划请求
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        
        # 设置目标位姿约束
        constraints = Constraints()
        
        # 位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'gripper_base'
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # 约束区域
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm 容差
        
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        
        # 姿态约束
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = 'base_link'
        orient_constraint.link_name = 'gripper_base'
        orient_constraint.orientation = pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orient_constraint)
        goal.request.goal_constraints.append(constraints)
        
        # 发送目标
        future = self.move_group_client.send_goal_async(goal)
        
        # 等待结果
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None:
            self.get_logger().error('运动规划失败')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        return True

    def state_machine_callback(self):
        """状态机主循环"""
        if self.state == GraspState.IDLE:
            pass
        
        elif self.state == GraspState.DETECTING:
            if self.target_pose is not None:
                self.get_logger().info('检测到目标，开始接近')
                self.state = GraspState.APPROACHING
        
        elif self.state == GraspState.APPROACHING:
            # 计算接近位置（目标上方）
            approach_pose = Pose()
            approach_pose.position.x = self.target_pose.pose.position.x
            approach_pose.position.y = self.target_pose.pose.position.y
            approach_pose.position.z = (self.target_pose.pose.position.z + 
                                        self.approach_dist)
            # 末端朝下的姿态
            approach_pose.orientation.x = 1.0
            approach_pose.orientation.y = 0.0
            approach_pose.orientation.z = 0.0
            approach_pose.orientation.w = 0.0
            
            self.get_logger().info('移动到接近位置...')
            self.control_gripper(False)  # 打开夹爪
            
            # 这里简化处理，实际应该异步执行
            # success = await self.move_to_pose(approach_pose)
            self.state = GraspState.GRASPING
        
        elif self.state == GraspState.GRASPING:
            # 下降到抓取位置
            grasp_pose = Pose()
            grasp_pose.position.x = self.target_pose.pose.position.x
            grasp_pose.position.y = self.target_pose.pose.position.y
            grasp_pose.position.z = (self.target_pose.pose.position.z + 
                                     self.grasp_offset)
            grasp_pose.orientation.x = 1.0
            grasp_pose.orientation.y = 0.0
            grasp_pose.orientation.z = 0.0
            grasp_pose.orientation.w = 0.0
            
            self.get_logger().info('下降抓取...')
            # success = await self.move_to_pose(grasp_pose)
            
            # 关闭夹爪
            self.control_gripper(True)
            self.state = GraspState.LIFTING
        
        elif self.state == GraspState.LIFTING:
            # 提起物体
            lift_pose = Pose()
            lift_pose.position.x = self.target_pose.pose.position.x
            lift_pose.position.y = self.target_pose.pose.position.y
            lift_pose.position.z = (self.target_pose.pose.position.z + 
                                    self.lift_height)
            lift_pose.orientation.x = 1.0
            lift_pose.orientation.y = 0.0
            lift_pose.orientation.z = 0.0
            lift_pose.orientation.w = 0.0
            
            self.get_logger().info('提起物体...')
            self.state = GraspState.MOVING
        
        elif self.state == GraspState.MOVING:
            # 移动到放置位置上方
            move_pose = Pose()
            move_pose.position.x = self.place_pos[0]
            move_pose.position.y = self.place_pos[1]
            move_pose.position.z = self.place_pos[2] + self.lift_height
            move_pose.orientation.x = 1.0
            move_pose.orientation.y = 0.0
            move_pose.orientation.z = 0.0
            move_pose.orientation.w = 0.0
            
            self.get_logger().info('移动到放置位置...')
            self.state = GraspState.PLACING
        
        elif self.state == GraspState.PLACING:
            # 下降到放置位置
            place_pose = Pose()
            place_pose.position.x = self.place_pos[0]
            place_pose.position.y = self.place_pos[1]
            place_pose.position.z = self.place_pos[2]
            place_pose.orientation.x = 1.0
            place_pose.orientation.y = 0.0
            place_pose.orientation.z = 0.0
            place_pose.orientation.w = 0.0
            
            self.get_logger().info('放置物体...')
            self.state = GraspState.RELEASING
        
        elif self.state == GraspState.RELEASING:
            # 打开夹爪
            self.control_gripper(False)
            self.get_logger().info('释放物体')
            self.state = GraspState.RETURNING
        
        elif self.state == GraspState.RETURNING:
            self.get_logger().info('返回初始位置')
            self.target_pose = None
            self.state = GraspState.IDLE
            self.get_logger().info('抓取任务完成!')


def main(args=None):
    rclpy.init(args=args)
    node = VisualGraspController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()