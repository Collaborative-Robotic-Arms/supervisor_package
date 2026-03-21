#!/usr/bin/env python3
"""
Mock Detection Node
Provides brick detection service without requiring actual vision system.
Automatically adapts to the 'test_scenario' launch parameter.
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
        
        # --- READ LAUNCH PARAMETER ---
        self.declare_parameter('test_scenario', 1)
        
        self.srv = self.create_service(
            DetectBricks,
            'detect_bricks',
            self.detect_bricks_callback
        )
        
        self.get_logger().info('Mock Detection Node started - serving /detect_bricks')
    
    def detect_bricks_callback(self, request, response):
        """
        Generate mock brick detections based on the active test scenario.
        """
        scenario = self.get_parameter('test_scenario').value
        self.get_logger().info(f'Received detect_bricks request for Scenario {scenario}')
        
        # Constants
        BRICK_TYPES = [0, 1, 2, 3]  # I=0, L=1, T=2, Z=3
        BRICK_TYPE_NAMES = ["I_BRICK", "L_BRICK", "T_BRICK", "Z_BRICK"]
        ABB = 0
        AR4 = 1
        
        bricks = []
        
        # =====================================================================
        # SCENARIO 1: PARALLEL NO COLLISION (Y = +/- 0.30)
        # =====================================================================
        if scenario == 1:
            for i in range(4): # Matches GUI node count
                brick = Brick()
                brick.header = Header(frame_id="abb_table", stamp=self.get_clock().now().to_msg())
                brick.id = i
                brick.type = BRICK_TYPES[i % 4]
                brick.side = AR4 if i % 2 == 0 else ABB
                
                brick.pose = Pose()
                if brick.side == AR4:
                    brick.pose.position = Point(x=0.7, y=0.20, z=0.2)
                    brick.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                else:
                    brick.pose.position = Point(x=0.40, y=-0.30, z=0.3)
                    brick.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                bricks.append(brick)

        # =====================================================================
        # SCENARIO 2: PARALLEL WITH COLLISION RISK (Y = +/- 0.10)
        # =====================================================================
        elif scenario == 2:
            for i in range(5): # Matches GUI node count
                brick = Brick()
                brick.header = Header(frame_id="abb_table", stamp=self.get_clock().now().to_msg())
                brick.id = i
                brick.type = BRICK_TYPES[i % 4]
                brick.side = AR4 if i % 2 == 0 else ABB
                
                brick.pose = Pose()
                if brick.side == AR4:
                    brick.pose.position = Point(x=0.7, y=0.10, z=0.2)
                    brick.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                else:
                    brick.pose.position = Point(x=0.40, y=-0.10, z=0.3)
                    brick.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                bricks.append(brick)

        # =====================================================================
        # SCENARIO 3: HANDOVER (AR4 -> ABB)
        # =====================================================================
        elif scenario == 3:
            brick = Brick()
            brick.header = Header(frame_id="abb_table", stamp=self.get_clock().now().to_msg())
            brick.id = 99
            brick.type = BRICK_TYPES[0]
            brick.side = AR4
            
            brick.pose = Pose()
            brick.pose.position = Point(x=0.7, y=0.10, z=0.2)
            brick.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            bricks.append(brick)

        # =====================================================================
        # SCENARIO 4: HANDOVER (ABB -> AR4)
        # =====================================================================
        elif scenario == 4:
            brick = Brick()
            brick.header = Header(frame_id="abb_table", stamp=self.get_clock().now().to_msg())
            brick.id = 98
            brick.type = BRICK_TYPES[0]
            brick.side = ABB
            
            brick.pose = Pose()
            brick.pose.position = Point(x=0.40, y=-0.10, z=0.3)
            brick.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            bricks.append(brick)

        else:
            self.get_logger().warn(f'Unknown scenario {scenario}. Returning empty array.')

        # Log out what the camera "sees"
        for i, b in enumerate(bricks):
            side_name = "AR4" if b.side == AR4 else "ABB"
            self.get_logger().info(
                f"  Camera saw Brick {b.id} on {side_name} at ({b.pose.position.x:.2f}, {b.pose.position.y:.2f}, {b.pose.position.z:.2f})"
            )

        response.bricks = bricks
        
        # =====================================================================
        # STATIC MID-AIR HANDOVER POSE 
        # (Where the arms should meet in the center of the table)
        # =====================================================================
        response.handover_pose = Pose()
        response.handover_pose.position = Point(x=0.55, y=0.0, z=0.4)
        # This quaternion doesn't matter much here since the Supervisor overwrites 
        # it with the explicit AR4/ABB grip orientations, but we set a clean default
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