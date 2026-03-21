#!/usr/bin/env python3
"""
Mock Grasping Pipeline Node
Provides grasp point service without requiring an actual grasping model.
Automatically adapts to the 'test_scenario' launch parameter.
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
        
        # --- READ LAUNCH PARAMETER ---
        self.declare_parameter('test_scenario', 1)
        
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
            
        scenario = self.get_parameter('test_scenario').value
        
        grasp = GraspPoint()
        grasp.header = Header()
        grasp.header.frame_id = "abb_table"
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.brick_id = brick_id
        grasp.pose = Pose()
        
        is_ar4_side = (brick_id % 2 == 0)

        # =====================================================================
        # SCENARIO 1: PARALLEL NO COLLISION
        # =====================================================================
        if scenario == 1:
            if is_ar4_side:
                grasp.pose.position = Point(x=0.7, y=0.20, z=0.2)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                grasp.pose.position = Point(x=0.40, y=-0.30, z=0.3)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)

        # =====================================================================
        # SCENARIO 2: PARALLEL WITH COLLISION RISK
        # =====================================================================
        elif scenario == 2:
            if is_ar4_side:
                grasp.pose.position = Point(x=0.7, y=0.10, z=0.2)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                grasp.pose.position = Point(x=0.40, y=-0.10, z=0.3)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)

        # =====================================================================
        # SCENARIO 3: HANDOVER (AR4 -> ABB)
        # =====================================================================
        elif scenario == 3:
            # If AR4 is asking, it's doing the initial pick from the table
            if is_ar4_side:
                grasp.pose.position = Point(x=0.7, y=0.10, z=0.2)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            # If ABB is asking, it's retrieving the brick from mid-air
            else:
                grasp.pose.position = Point(x=0.56, y=-0.1, z=0.4)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)

        # =====================================================================
        # SCENARIO 4: HANDOVER (ABB -> AR4)
        # =====================================================================
        elif scenario == 4:
            # If ABB is asking, it's doing the initial pick from the table
            if not is_ar4_side:
                grasp.pose.position = Point(x=0.40, y=-0.10, z=0.3)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            # If AR4 is asking, it's retrieving the brick from mid-air
            else:
                grasp.pose.position = Point(x=0.56, y=-0.1, z=0.3)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
       
        else:
            self.get_logger().warn(f'Unknown scenario {scenario}, defaulting to Scenario 1.')
            if is_ar4_side:
                grasp.pose.position = Point(x=0.65, y=0.30, z=0.2)
                grasp.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                grasp.pose.position = Point(x=0.40, y=-0.30, z=0.1)
                grasp.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)

        # Simulate dynamic grasp quality
        grasp.quality = random.uniform(0.7, 0.99)
        return grasp
    
    def get_grasp_callback(self, request, response):
        """
        Callback for grasp service request
        Returns: GraspPoint with pose and quality score
        """
        brick_index = request.brick_index
        
        self.get_logger().info(f'Received get_grasp request for brick {brick_index}')
        
        grasp = self._generate_grasp_for_brick(brick_index)
        response.grasp_point = grasp
        response.success = True
        
        scenario = self.get_parameter('test_scenario').value
        self.get_logger().info(
            f'[Scenario {scenario}] Grasping brick {brick_index}: quality={grasp.quality:.2f}, '
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