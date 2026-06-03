#!/usr/bin/env python3
"""
Sequential task runner for the dual-arm handover.

Sends a fixed sequence of ExecuteTask goals to the 'ar4_control' and
'abb_control' action servers. Each task blocks until the previous one has
finished (succeeded) before the next is dispatched. If any task fails or is
rejected, the sequence aborts.

Sequence:
    1. PICK              -> AR4   (placeholder pose, fill in later)
    2. INTERMEDIATE_GIVE -> AR4   (0.623 0.06 0.23 / 0.707 0.707 0 0)
    3. INTERMEDIATE_TAKE -> ABB   (0.55 0 0.32 / 0 1 0 0)
    4. RELEASE           -> AR4   (no pose needed)
    5. PLACE             -> ABB   (placeholder pose, fill in later)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from dual_arms_msgs.action import ExecuteTask


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

        self.ar4_client = ActionClient(self, ExecuteTask, 'ar4_control')
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control')

    def send_task(self, client, server_name, task_type, target_pose=None):
        """Send a single goal and block until it finishes.

        Returns True if the task succeeded, False otherwise.
        """
        self.get_logger().info(f"Waiting for '{server_name}' action server...")
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f"Action server '{server_name}' not available.")
            return False

        goal = ExecuteTask.Goal()
        goal.task_type = task_type
        if target_pose is not None:
            goal.target_pose = target_pose

        self.get_logger().info(f"[{server_name}] Sending task '{task_type}'...")

        # --- send goal ---
        send_goal_future = client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"[{server_name}] Goal '{task_type}' was REJECTED.")
            return False

        self.get_logger().info(f"[{server_name}] Goal '{task_type}' accepted, executing...")

        # --- wait for result ---
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()

        if wrapped is None:
            self.get_logger().error(f"[{server_name}] No result for '{task_type}'.")
            return False

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"[{server_name}] Task '{task_type}' did not succeed "
                f"(status={wrapped.status}, msg='{wrapped.result.error_message}').")
            return False

        result = wrapped.result
        if not result.success:
            self.get_logger().error(
                f"[{server_name}] Task '{task_type}' reported failure: "
                f"'{result.error_message}'.")
            return False

        self.get_logger().info(f"[{server_name}] Task '{task_type}' SUCCEEDED.")
        return True

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"  feedback: progress={fb.progress:.2f} status='{fb.current_status}'")

    def run_sequence(self):
        """Run the full sequence. Stops at the first failure."""

        # ------------------------------------------------------------------
        # 1. PICK  (AR4)  -- PLACEHOLDER POSE: fill in x, y, z + orientation
        # ------------------------------------------------------------------
        pick_pose = make_pose(
            0.75, -0.22, 0.14,      # TODO: x, y, z
            0.707, 0.707, 0.0, 0.0  # TODO: orientation (qx, qy, qz, qw)
        )
        if not self.send_task(self.ar4_client, 'ar4_control', 'PICK', pick_pose):
            return False

        # ------------------------------------------------------------------
        # 2. INTERMEDIATE_GIVE  (AR4)
        # ------------------------------------------------------------------
        give_pose = make_pose(0.623, 0.06, 0.23, 0.707, 0.707, 0.0, 0.0)
        if not self.send_task(self.ar4_client, 'ar4_control', 'INTERMEDIATE_GIVE', give_pose):
            return False

        # ------------------------------------------------------------------
        # 3. INTERMEDIATE_TAKE  (ABB)
        # ------------------------------------------------------------------
        take_pose = make_pose(0.55, 0.0, 0.32, 0.0, 1.0, 0.0, 0.0)
        if not self.send_task(self.abb_client, 'abb_control', 'INTERMEDIATE_TAKE', take_pose):
            return False

        # ------------------------------------------------------------------
        # 4. RELEASE  (AR4)  -- no pose needed
        # ------------------------------------------------------------------
        if not self.send_task(self.ar4_client, 'ar4_control', 'RELEASE'):
            return False

        # ------------------------------------------------------------------
        # 5. PLACE  (ABB)  -- PLACEHOLDER POSE: fill in x, y, z + orientation
        # ------------------------------------------------------------------
        place_pose = make_pose(
            0.4, 0.0, 0.22,      # TODO: x, y, z
            0.0, 1.0, 0.0, 0.0  # TODO: orientation (qx, qy, qz, qw)
        )
        if not self.send_task(self.abb_client, 'abb_control', 'PLACE', place_pose):
            return False

        self.get_logger().info("Full sequence completed successfully.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequenceRunner()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
