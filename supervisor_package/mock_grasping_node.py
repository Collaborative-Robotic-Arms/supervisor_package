#!/usr/bin/env python3
"""
Mock Grasping Pipeline Node
Provides grasp point service without requiring actual grasping model
"""

import rclpy
from rclpy.node import Node
from dual_arms_msgs.srv import GetGrasp
from dual_arms_msgs.msg import GraspPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import random


class MockGraspingNode(Node):
    """Mock Grasping Pipeline Service Provider"""
    
    def __init__(self):
        super().__init__('mock_grasping_node')
        
        self.srv = self.create_service(
            GetGrasp,
            'grasp/get_grasp_point',
            self.get_grasp_callback
        )
        
        self.get_logger().info('Mock Grasping Pipeline Node started - serving /grasp/get_grasp_point')
    
    def _generate_grasp_for_brick(self, brick_index: str):
        try:
            brick_id = int(brick_index)
        except ValueError:
            brick_id = 0
        
        grasp = GraspPoint()
        grasp.header = Header()
        grasp.header.frame_id = "abb_table"
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.brick_id = brick_id
        
        is_ar4_side = (brick_id % 2 == 0)
        grasp.pose = Pose()
        
        # Split the grasp coordinates based on which arm is picking
        if is_ar4_side:
            grasp.pose.position = Point(x=0.75, y=0.10, z=0.1)
            grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        else:
            grasp.pose.position = Point(x=0.40, y=-0.10, z=0.1)
            grasp.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        
        grasp.quality = random.uniform(0.7, 0.99)
        return grasp
    
    def get_grasp_callback(self, request, response):
        """
        Callback for grasp service request
        
        Returns:
            GraspPoint with pose and quality score
        """
        brick_index = request.brick_index
        
        self.get_logger().info(f'Received get_grasp request for brick {brick_index}')
        
        grasp = self._generate_grasp_for_brick(brick_index)
        response.grasp_point = grasp
        response.success = True
        
        self.get_logger().info(
            f'Grasping brick {brick_index}: quality={grasp.quality:.2f}, '
            f'pose=({grasp.pose.position.x:.3f}, {grasp.pose.position.y:.3f}, {grasp.pose.position.z:.3f})'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGraspingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
