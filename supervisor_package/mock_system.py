#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from supervisor_package.srv import GetAssemblyPlan, DetectBricks
from supervisor_package.action import ExecuteTask # Keep only ABB action
from supervisor_package.msg import SuperBrick as Brick 
from geometry_msgs.msg import Pose
from rclpy.action import ActionServer

class MockSystem(Node):
    def __init__(self):
        super().__init__('mock_system')
        
        # 1. Services: We NEED these to start the sequence
        self.plan_srv = self.create_service(GetAssemblyPlan, 'get_assembly_plan', self.plan_callback)
        self.detect_srv = self.create_service(DetectBricks, 'detect_bricks', self.detect_callback)
        
        # 2. ABB Action: We keep this mocked so the AR4 sequence can "finish" handover
        self.abb_server = ActionServer(
            self, ExecuteTask, 'abb_control', self.execute_abb_callback)

        self.get_logger().info("MOCK SYSTEM: Providing Plan & Vision. AR4 Actions are DISABLED (use real C++ nodes).")

    def plan_callback(self, request, response):
        self.get_logger().info("GUI: Sending Assembly Plan for Brick 1...")
        b1 = Brick()
        b1.id = 1
        b1.type = "red_brick"
        b1.start_side = "AR4"
        b1.target_side = "ABB"
        response.plan = [b1] 
        response.success = True
        return response

    def detect_callback(self, request, response):
        self.get_logger().info("VISION: Sending Detected Brick Pose...")
        b1 = Brick()
        b1.id = 1
        # Target coordinates for your AR4 to move to in simulation
        b1.pose.position.x = 0.70
        b1.pose.position.y = 0.05
        b1.pose.position.z = 0.15
        response.bricks = [b1]
        
        hp = Pose()
        hp.position.x = 0.5
        hp.position.y = 0.05
        hp.position.z = 0.30
        response.handover_pose = hp
        
        response.success = True
        return response

    async def execute_abb_callback(self, goal_handle):
        self.get_logger().info(f'MOCK: Receiving brick from AR4...')
        goal_handle.succeed()
        return ExecuteTask.Result()

def main():
    rclpy.init()
    node = MockSystem()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()