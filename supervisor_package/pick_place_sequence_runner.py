#!/usr/bin/env python3
"""
Generalized Parallel task runner with MTC Collision Avoidance Integration.

Listens to the /zone_status topic. If the arms cross the min_arm_separation 
threshold, it cancels the parallel goals, yields control to MTC, and seamlessly
resumes the sequence once MTC safely finishes the movement.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time

# Custom Interfaces
from dual_arms_msgs.action import ExecuteTask
from dual_arms_msgs.srv import ResolveCollision


def make_pose(x, y, z, qx, qy, qz, qw):
    """Build a geometry_msgs/Pose from position + quaternion components."""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


class SequenceRunner(Node):
    def __init__(self):
        super().__init__('sequence_runner')

        self.cb_group = ReentrantCallbackGroup()

        # Action Clients
        self.ar4_client = ActionClient(self, ExecuteTask, 'ar4_control', callback_group=self.cb_group)
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        # MTC Safety Integration
        self.zone_sub = self.create_subscription(
            String, '/zone_status', self.zone_status_callback, 10, callback_group=self.cb_group
        )
        self.mtc_resolve_client = self.create_client(
            ResolveCollision, 'mtc_controller/resolve_collision', callback_group=self.cb_group
        )

        # State Tracking for Interruption
        self.ar4_goal_handle = None
        self.abb_goal_handle = None
        self.ar4_active_target = None
        self.abb_active_target = None
        
        self.mtc_active = False
        self.mtc_task_future = None
        self.mtc_last_run_time = 0.0  # Used for the safety cooldown

        self.timer = self.create_timer(1.0, self.start_sequence, callback_group=self.cb_group)

    def zone_status_callback(self, msg):
        """Listens for the danger threshold from zone_detection_manager."""
        if "COLLISION_WARNING" in msg.data and not self.mtc_active:
            
            # THE FIX: 3-Second Cooldown. Gives the arms time to pull away 
            # after MTC finishes without instantly triggering again.
            if time.time() - self.mtc_last_run_time < 3.0:
                return 

            self.get_logger().warn("⚠️ COLLISION WARNING DETECTED! Interrupting sequence!")
            self.mtc_active = True
            
            # 1. Slam the brakes on both standard action servers
            if self.ar4_goal_handle: 
                self.ar4_goal_handle.cancel_goal_async()
            if self.abb_goal_handle: 
                self.abb_goal_handle.cancel_goal_async()
            
            # 2. Fire the MTC resolution service and SAVE its progress
            self.mtc_task_future = self.executor.create_task(self.trigger_mtc_resolution())

    async def trigger_mtc_resolution(self):
        """Asks MTC to take over and safely move the arms to their targets."""
        self.get_logger().info("Handing over control to MTC for safe resolution...")
        if not self.mtc_resolve_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("MTC Resolution service not available!")
            self.mtc_active = False
            return False
        
        req = ResolveCollision.Request()
        req.ar4_target_pose = self.ar4_active_target if self.ar4_active_target else Pose()
        req.abb_target_pose = self.abb_active_target if self.abb_active_target else Pose()
        
        result = await self.mtc_resolve_client.call_async(req)
        
        if result.success:
            self.get_logger().info("✅ MTC successfully resolved the collision and reached targets.")
        else:
            self.get_logger().error(f"❌ MTC failed to resolve: {result.message}")
        
        # Reset trackers and activate cooldown timer
        self.mtc_last_run_time = time.time()
        self.mtc_active = False
        return result.success

    async def start_sequence(self):
        self.timer.cancel()
        await self.run_sequence()

    async def send_task(self, client, server_name, task_type, target_pose=None):
        """Async method to send a single goal and yield until it finishes."""
        if not client.wait_for_server(timeout_sec=15.0):
            return False

        goal = ExecuteTask.Goal()
        goal.task_type = task_type
        if target_pose is not None:
            goal.target_pose = target_pose

        # --- Track our targets so MTC knows where we were going if interrupted ---
        if target_pose is not None:
            if server_name == 'ar4_control': self.ar4_active_target = target_pose
            if server_name == 'abb_control': self.abb_active_target = target_pose

        self.get_logger().info(f"[{server_name}] Sending task '{task_type}'...")

        goal_future = client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        goal_handle = await goal_future

        if goal_handle is None or not goal_handle.accepted:
            return False

        if server_name == 'ar4_control': self.ar4_goal_handle = goal_handle
        if server_name == 'abb_control': self.abb_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        wrapped = await result_future

        if server_name == 'ar4_control': self.ar4_goal_handle = None
        if server_name == 'abb_control': self.abb_goal_handle = None

        if wrapped is None:
            return False

        # --- THE FIX: SEAMLESS MTC HANDOFF ---
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if self.mtc_active and self.mtc_task_future:
                self.get_logger().warn(f"[{server_name}] Yielding to MTC. Pausing sequence until MTC finishes...")
                
                # Wait right here until MTC finishes!
                mtc_success = await self.mtc_task_future
                
                if mtc_success:
                    self.get_logger().info(f"[{server_name}] MTC reached the '{task_type}' target for us! Resuming sequence.")
                    return True  # Pretend the goal succeeded so the workflow continues
                else:
                    return False
            else:
                self.get_logger().warn(f"[{server_name}] Task '{task_type}' was manually CANCELED.")
                return False

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"[{server_name}] Task failed: {wrapped.result.error_message}")
            return False

        self.get_logger().info(f"[{server_name}] Task '{task_type}' SUCCEEDED.")
        return True

    def _feedback_cb(self, feedback_msg):
        pass # Silenced the spammy progress feedback for cleaner logs

    async def ar4_workflow(self):
        """Complete independent sequence for the AR4 arm."""
        ar4_pick_pose = make_pose(0.75, -0.22, 0.14, 0.707, 0.707, 0.0, 0.0)
        ar4_place_pose = make_pose(0.623, 0.06, 0.23, 0.707, 0.707, 0.0, 0.0)
        
        if not await self.send_task(self.ar4_client, 'ar4_control', 'PICK', ar4_pick_pose): return False
        if not await self.send_task(self.ar4_client, 'ar4_control', 'PLACE', ar4_place_pose): return False
        
        self.get_logger().info("AR4 finished PLACE. Returning HOME...")
        if not await self.send_task(self.ar4_client, 'ar4_control', 'HOME', None): return False
        
        self.get_logger().info("✅ AR4 Workflow Complete!")
        return True

    async def abb_workflow(self):
        """Complete independent sequence for the ABB arm."""
        abb_pick_pose = make_pose(0.35, 0.0, 0.32, 0.0, 1.0, 0.0, 0.0)
        abb_place_pose = make_pose(0.4, 0.0, 0.22, 0.0, 1.0, 0.0, 0.0)
        
        if not await self.send_task(self.abb_client, 'abb_control', 'PICK', abb_pick_pose): return False
        if not await self.send_task(self.abb_client, 'abb_control', 'PLACE', abb_place_pose): return False
        
        self.get_logger().info("ABB finished PLACE. Returning HOME...")
        if not await self.send_task(self.abb_client, 'abb_control', 'HOME', None): return False
        
        self.get_logger().info("✅ ABB Workflow Complete!")
        return True

    async def run_sequence(self):
        """Deploy both workflows completely independently."""
        self.get_logger().info("Starting independent parallel workflows...")
        
        t_ar4 = self.executor.create_task(self.ar4_workflow())
        t_abb = self.executor.create_task(self.abb_workflow())
        
        ar4_success = await t_ar4
        abb_success = await t_abb
        
        if ar4_success and abb_success:
            self.get_logger().info("Both independent sequences completed successfully.")
        else:
            self.get_logger().warn("Sequence aborted due to failure.")

def main(args=None):
    rclpy.init(args=args)
    node = SequenceRunner()
    
    executor = MultiThreadedExecutor()
    node.executor = executor
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()