#!/usr/bin/env python3
"""
Mock Detection Node
Provides brick detection service without requiring actual vision system
"""

import rclpy
from rclpy.node import Node
from dual_arms_msgs.srv import DetectBricks
from dual_arms_msgs.msg import Brick
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import random


class MockDetectionNode(Node):
    """Mock Detection Service Provider"""
    
    def __init__(self):
        super().__init__('mock_detection_node')
        
        self.srv = self.create_service(
            DetectBricks,
            'detect_bricks',
            self.detect_bricks_callback
        )
        
        self.get_logger().info('Mock Detection Node started - serving /detect_bricks')
    
    def detect_bricks_callback(self, request, response):
        """
        Generate mock brick detections
        
        Returns:
            5-7 bricks randomly distributed between AR4 and ABB starting positions
        """
        self.get_logger().info('Received detect_bricks request')
        
        # Brick type constants
        BRICK_TYPES = [0, 1, 2, 3]  # I_BRICK=0, L_BRICK=1, T_BRICK=2, Z_BRICK=3
        BRICK_TYPE_NAMES = ["I_BRICK", "L_BRICK", "T_BRICK", "Z_BRICK"]
        
        # Side constants
        ABB = 0
        AR4 = 1
        
        # Generate 5-7 mock bricks
        num_bricks = random.randint(5, 7)
        bricks = []
        
        for i in range(num_bricks):
            brick = Brick()
            brick.header = Header()
            brick.header.stamp = self.get_clock().now().to_msg()
            brick.header.frame_id = "abb_table"
            
            brick.id = i
            
            # Random brick type (0-3)
            brick.type = random.choice(BRICK_TYPES)
            brick_type_name = BRICK_TYPE_NAMES[brick.type]
            
            # Alternate between AR4 and ABB as starting position
            # Alternate between AR4 and ABB as starting position
            brick.side = AR4 if i % 2 == 0 else ABB
            side_name = "AR4" if brick.side == AR4 else "ABB"
            
            # Match the detection coordinates to the grasp coordinates
            brick.pose = Pose()
            if brick.side == AR4:
                brick.pose.position = Point(x=0.75, y=0.10, z=0.1)
                brick.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                brick.pose.position = Point(x=0.40, y=-0.10, z=0.1)
                brick.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
            
            bricks.append(brick)
            self.get_logger().info(
                f"  Brick {i}: {brick_type_name} on {side_name} at ({brick.pose.position.x:.2f}, {brick.pose.position.y:.2f}, {brick.pose.position.z:.2f})"
            )
        
        response.bricks = bricks
        
        # Mock handover pose - neutral position between both arms
        response.handover_pose = Pose()
        response.handover_pose.position = Point(x=0.55, y=0.0, z=0.4)
        response.handover_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        response.success = True
        
        self.get_logger().info(f'Returning {len(bricks)} detected bricks')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
