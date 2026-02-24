#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .gripper_manager import GripperManager

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        # Change this to False if testing on real hardware
        self.use_sim = True 
        self.gripper = GripperManager(self, use_sim=self.use_sim)
        
        self.get_logger().info(f"--- GRIPPER TESTER READY (SIM: {self.use_sim}) ---")
        self.run_test_sequence()

    def run_test_sequence(self):
        # 1. Test AR4 Open/Close
        self.get_logger().info("Testing AR4 Gripper...")
        self.gripper.set_ar4_grip('CLOSE')
        self.gripper.set_ar4_grip('OPEN')

        # 2. Test ABB Open/Close
        self.get_logger().info("Testing ABB Gripper...")
        self.gripper.set_abb_grip('CLOSE')
        self.gripper.set_abb_grip('OPEN')
        
        self.get_logger().info("Test Sequence Complete.")

def main(args=None):
    rclpy.init(args=args)
    node = GripperTester()
    rclpy.shutdown()

if __name__ == '__main__':
    main()