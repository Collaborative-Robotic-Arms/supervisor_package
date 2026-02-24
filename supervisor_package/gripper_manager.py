#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperManager:
    """
    Unified Manager to handle gripper execution for both 
    Simulation (Gazebo) and Physical Hardware.
    """
    def __init__(self, node: Node, use_sim: bool = True):
        """
        node: The parent supervisor node.
        use_sim: Toggle between Gazebo plugins and Hardware drivers.
        """
        self.node = node
        self.use_sim = use_sim
        
        # --- COMMON TRAJECTORY PUBLISHERS (Visuals/Drivers) ---
        self.ar4_traj_pub = node.create_publisher(
            JointTrajectory, '/ar4_gripper_controller/joint_trajectory', 10)
            
        self.abb_traj_pub = node.create_publisher(
            JointTrajectory, '/irb120_gripper_controller/joint_trajectory', 10)

        # --- MODE-SPECIFIC CONFIGURATION ---
        if self.use_sim:
            # Publishers for Gazebo "Sticky" logic
            self.ar4_lock_pub = node.create_publisher(Bool, '/ar4/gripper/lock_trigger', 10)
            self.abb_lock_pub = node.create_publisher(Bool, '/abb/gripper/lock_trigger', 10)
            self.node.get_logger().info("[Gripper] Initialized in SIMULATION mode.")
        else:
            # Bridge Topic for ABB Hardware (Matches end_effector_bridge subscriber)
            # The C++ bridge node listens to this topic to trigger RAPID services
            self.abb_hw_bridge_pub = node.create_publisher(Bool, '/gripper_command', 10)
            
            # Publisher for AR4 hardware
            self.ar4_hw_pub = node.create_publisher(String, '/ar4/gripper/command', 10)
            self.node.get_logger().info("[Gripper] Initialized in HARDWARE mode with Bridge Topic.")

    def set_ar4_grip(self, state: str):
        """state: 'OPEN' or 'CLOSE'"""
        # 1. Send Visual/Joint command
        traj = JointTrajectory()
        traj.joint_names = ['ar4_gripper_jaw1_joint', 'ar4_gripper_jaw2_joint']
        point = JointTrajectoryPoint()
        
        is_closing = (state == 'CLOSE')
        point.positions = [0.0, 0.0] if is_closing else [0.015, 0.015]
        point.time_from_start.sec = 1
        traj.points = [point]
        
        self.node.get_logger().info(f"[Gripper] AR4 {state} command sent.")
        self.ar4_traj_pub.publish(traj)
        
        # 2. Handle Physics/Actuation logic
        time.sleep(1.2) # Wait for physical/visual movement
        
        if self.use_sim:
            self._trigger_lock(self.ar4_lock_pub, is_closing)
        else:
            msg = String()
            msg.data = "GRIP_ON" if is_closing else "GRIP_OFF"
            self.ar4_hw_pub.publish(msg)

    def set_abb_grip(self, state: str):
        """state: 'OPEN' or 'CLOSE'"""
        # 1. Send Visual/Joint command (Trajectory for RViz/Gazebo)
        traj = JointTrajectory()
        traj.joint_names = ['gripper_ABB_Gripper_Finger_1_Joint', 'gripper_ABB_Gripper_Finger_2_Joint']
        point = JointTrajectoryPoint()
        
        is_opening = (state == 'OPEN')
        point.positions = [0.0135, 0.0135] if is_opening else [0.0, 0.0]
        point.time_from_start.sec = 1
        traj.points = [point]
        
        self.node.get_logger().info(f"[Gripper] ABB {state} command sent.")
        self.abb_traj_pub.publish(traj)
        
        # 2. Handle Physics/Actuation logic
        if self.use_sim:
            time.sleep(1.2) # Sync delay for simulation visuals
            self._trigger_lock(self.abb_lock_pub, not is_opening)
        else:
            # HARDWARE MODE: Publish to the bridge topic
            # The bridge node receives this and calls the SetRAPIDBool service
            msg = Bool()
            msg.data = is_opening # True for OPEN, False for CLOSE
            
            self.node.get_logger().info(f"[Gripper] Publishing {state} to /gripper_command bridge topic.")
            self.abb_hw_bridge_pub.publish(msg)

    def _trigger_lock(self, publisher, state: bool):
        """Internal helper for Simulation Sticky Logic."""
        msg = Bool()
        msg.data = state
        publisher.publish(msg)