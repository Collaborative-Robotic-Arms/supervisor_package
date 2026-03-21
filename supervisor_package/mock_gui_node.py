#!/usr/bin/env python3
"""
Mock GUI Node
Provides assembly plan service without requiring actual GUI.
Supports multiple test scenarios driven by ROS 2 launch parameters.
"""

import rclpy
from rclpy.node import Node
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.msg import SuperBrick
from geometry_msgs.msg import Pose, Point, Quaternion

class MockGUINode(Node):
    """Mock GUI Service Provider"""
    
    def __init__(self):
        super().__init__('mock_gui_node')
        
        # Declare the parameter injected by full_system.launch.py
        self.declare_parameter('test_scenario', 1)
        
        self.srv = self.create_service(
            GetAssemblyPlan,
            'get_assembly_plan',
            self.get_assembly_plan_callback
        )
        self.plan_dispatched = False
        self.get_logger().info('Mock GUI Node started - serving /get_assembly_plan')
    
    def _scenario_1_parallel_no_collision(self):
        """
        SCENARIO 1: True Parallel (Safe Distances)
        Deduction: We take the parallel sequence but force the Y-coordinates 
        to be far apart (AR4 at +0.30, ABB at -0.30) so they never trigger MTC.
        """
        plan = []
        for i in range(4):
            brick = SuperBrick()
            brick.id = i
            brick.type = f"brick_model_v{i % 3 + 1}"
            brick.start_side = "AR4" if i % 2 == 0 else "ABB"
            brick.target_side = "TOP"
            
            # Keep arms on completely opposite sides of the table
            if brick.start_side == "AR4":
                brick.pickup_pose.position = Point(x=0.7, y=0.20, z=0.2)
                brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                brick.place_pose.position = Point(x=0.65, y=0.20, z=0.2)
                brick.place_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                brick.pickup_pose.position = Point(x=0.40, y=-0.30, z=0.3)
                brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                brick.place_pose.position = Point(x=0.45, y=-0.30, z=0.3)
                brick.place_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            plan.append(brick)
        return plan

    def _scenario_2_parallel_collision(self):
        """
        SCENARIO 2: True Parallel (With MTC Collision Risk)
        Exactly as you provided: 5 bricks placed close to the center to force 
        the MTC safe-resolution pipeline to trigger.
        """
        plan = []
        for i in range(5):
            brick = SuperBrick()
            brick.id = i
            brick.type = f"brick_model_v{i % 3 + 1}"
            brick.start_side = "AR4" if i % 2 == 0 else "ABB"
            brick.target_side = "TOP" if i >= 3 else "BOTTOM"
            
            if brick.start_side == "AR4":
                brick.pickup_pose.position = Point(x=0.7, y=0.10, z=0.2) 
                brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                brick.place_pose.position = Point(x=0.65, y=0.10, z=0.2)
                brick.place_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            else:
                brick.pickup_pose.position = Point(x=0.40, y=-0.10, z=0.3)
                brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                brick.place_pose.position = Point(x=0.55, y=-0.10, z=0.3 )
                brick.place_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
            plan.append(brick)
        return plan

    def _scenario_3_handover_ar4_to_abb(self):
        """
        SCENARIO 3: Handover AR4 -> ABB
        Exactly as provided in your tested handover file.
        """
        plan = []
        handover_brick = SuperBrick()
        handover_brick.id = 99
        handover_brick.type = "brick_model_handover"
        handover_brick.start_side = "AR4"
        handover_brick.target_side = "ABB" 
        
        handover_brick.pickup_pose.position = Point(x=0.7, y=0.10, z=0.2)
        handover_brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        
        handover_brick.place_pose.position = Point(x=0.50, y=-0.15, z=0.3)
        handover_brick.place_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        
        plan.append(handover_brick)
        return plan

    def _scenario_4_handover_abb_to_ar4(self):
        """
        SCENARIO 4: Handover ABB -> AR4
        The reverse of scenario 3.
        """
        plan = []
        handover_brick = SuperBrick()
        handover_brick.id = 98
        handover_brick.type = "brick_model_handover_rev"
        handover_brick.start_side = "ABB"
        handover_brick.target_side = "AR4" 
        
        handover_brick.pickup_pose.position = Point(x=0.40, y=-0.10, z=0.3)
        handover_brick.pickup_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        
        handover_brick.place_pose.position = Point(x=0.65, y=0.15, z=0.2)
        handover_brick.place_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        
        plan.append(handover_brick)
        return plan

    def get_assembly_plan_callback(self, request, response):
        """Return the assembly plan based on the launch parameter"""

        if self.plan_dispatched:
            response.plan = []
            response.success = True
            return response
        
        # Mark as dispatched so it never sends again
        self.plan_dispatched = True

        scenario = self.get_parameter('test_scenario').value
        self.get_logger().info(f'------------------------------------')
        self.get_logger().info(f'Loading Test Scenario {scenario}')
        
        if scenario == 1:
            plan = self._scenario_1_parallel_no_collision()
        elif scenario == 2:
            plan = self._scenario_2_parallel_collision()
        elif scenario == 3:
            plan = self._scenario_3_handover_ar4_to_abb()
        elif scenario == 4:
            plan = self._scenario_4_handover_abb_to_ar4()
        else:
            self.get_logger().warn(f'Unknown scenario {scenario}, defaulting to 1')
            plan = self._scenario_1_parallel_no_collision()
        
        response.plan = plan
        response.success = True
        
        self.get_logger().info(f'Returning assembly plan with {len(plan)} bricks:')
        for i, brick in enumerate(plan):
            self.get_logger().info(
                f"  Brick {i}: ID {brick.id} | {brick.start_side} -> {brick.target_side}"
            )
        self.get_logger().info(f'------------------------------------')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockGUINode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()