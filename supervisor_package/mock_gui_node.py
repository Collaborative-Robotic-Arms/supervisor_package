#!/usr/bin/env python3
"""
Mock GUI Node
Provides assembly plan service without requiring actual GUI
"""

import rclpy
from rclpy.node import Node
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.msg import SuperBrick
from geometry_msgs.msg import Pose, Point, Quaternion
import random


class MockGUINode(Node):
    """Mock GUI Service Provider"""
    
    def __init__(self):
        super().__init__('mock_gui_node')
        
        self.srv = self.create_service(
            GetAssemblyPlan,
            'get_assembly_plan',
            self.get_assembly_plan_callback
        )
        
        self.get_logger().info('Mock GUI Node started - serving /get_assembly_plan')
        self.plan_requested = False
    
    def _create_default_plan(self):
        """Create a default assembly plan with 5 bricks"""
        plan = []
        
        # Create a sequence of bricks alternating between AR4 and ABB pickups
        for i in range(5):
            brick = SuperBrick()
            brick.id = i
            brick.type = f"brick_model_v{i % 3 + 1}"
            
            # Alternate which arm picks: AR4 -> ABB -> AR4
            brick.start_side = "AR4" if i % 2 == 0 else "ABB"
            
            # Bricks go to assembly structure
            brick.target_side = "TOP" if i >= 3 else "BOTTOM"
            
            # --- PICKUP POSES ---
            if brick.start_side == "AR4":
                # Keep AR4 where it is (it works perfectly)
                brick.pickup_pose.position = Point(x=0.65, y=0.10, z=0.2) 
                brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                # Move ABB pickup closer to the ABB base (X=0)
                brick.pickup_pose = Pose()
                brick.pickup_pose.position = Point(x=0.40, y=-0.10, z=0.1) # <-- CHANGED TO 0.40
                brick.pickup_pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)

                
            # --- DESTINATION POSES ---
            # --- DESTINATION POSES ---
            brick.place_pose = Pose()
            
            # Match the placement orientation AND Y-offset to the arm that is holding it
            if brick.start_side == "AR4":
                # Place slightly to the left
                brick.place_pose.position = Point(x=0.65, y=0.10, z=0.2 + (i * 0.04))
                brick.place_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                # Place slightly to the right
                brick.place_pose.position = Point(x=0.55, y=-0.10, z=0.1 + (i * 0.04))
                brick.place_pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
            
            plan.append(brick)
        
        return plan
    
    def get_assembly_plan_callback(self, request, response):
        """
        Return assembly plan (can be parameterized)
        
        Returns:
            A list of SuperBrick objects representing the assembly sequence
        """
        self.get_logger().info('Received get_assembly_plan request')
        
        # Generate or return pre-configured plan
        plan = self._create_default_plan()
        
        response.plan = plan
        response.success = True
        
        self.get_logger().info(f'Returning assembly plan with {len(plan)} bricks')
        
        for i, brick in enumerate(plan):
            self.get_logger().info(
                f"  Brick {i}: {brick.type} - {brick.start_side} pickup -> {brick.target_side} placement"
            )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGUINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
