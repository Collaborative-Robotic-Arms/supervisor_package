#!/usr/bin/env python3
import rclpy
import time

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool

from dual_arms_msgs.action import ExecuteTask
from supervisor_package.action import MoveToPose
from action_msgs.msg import GoalStatus

class ABBController(Node):
    def __init__(self):
        super().__init__('abb_controller')
        self.cb_group = ReentrantCallbackGroup()

        # --- ARM ACTION CLIENT (ABB) ---
        self.move_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        # --- ACTION SERVER TO RECEIVE SUPERVISOR TASKS ---
        self.execute_task_server = ActionServer(
            self,
            ExecuteTask,
            'abb_controller/execute_task',
            execute_callback=self.execute_task_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('ABB Controller Ready.')
     
    async def send_action_goal(self, client, goal_msg):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server {client._action_name} not available!')
            return None

        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected by {client._action_name}')
            return None

        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return result.result
        else:
            self.get_logger().error(f'Action {client._action_name} failed with status: {result.status}')
            return None
    
    async def execute_abb_pick(self, grasp_point, goal_handle):
        self.get_logger().info(f'Starting ABB Pick')

        abb_pick_goal = ExecuteTask.Goal()
        abb_pick_goal.task_type = "PICK"

        # =========================
        # ORIENTATION ADJUSTMENT
        # =========================
        # Your current values
        current_x = grasp_point.orientation.z
        current_y = grasp_point.orientation.w

        # The magic number 0.7071 is 1/sqrt(2)
        factor = 0.70710678

        # To rotate +90 degrees (Counter-Clockwise)
        new_x = factor * (current_x - current_y)
        new_y = factor * (current_x + current_y)

        # To rotate -90 degrees (Clockwise)
        new_x = factor * (current_x + current_y)
        new_y = factor * (current_y - current_x)

        # Assign back to your message
        abb_pick_goal.target_pose = grasp_point
        abb_pick_goal.target_pose.orientation.x = new_x
        abb_pick_goal.target_pose.orientation.y = new_y
        abb_pick_goal.target_pose.orientation.z = 0.0
        abb_pick_goal.target_pose.orientation.w = 0.0

        action_result = await self.send_action_goal(self.move_client, abb_pick_goal)
        if action_result:   
            self.get_logger().info(f'ABB Picked')
            time.sleep(2)
        return action_result

    async def execute_abb_place(self, target_pose, goal_handle):
        self.get_logger().info('Starting ABB Place')

        abb_place_goal = ExecuteTask.Goal()
        abb_place_goal.task_type = "PLACE"

        # =========================
        # FINAL POSE (FROM SUPERVISOR)
        # =========================
        abb_place_goal.target_pose = target_pose

        # =========================
        # DEBUG LOGGER (UNCHANGED)
        # =========================
        p = abb_place_goal.target_pose.position
        o = abb_place_goal.target_pose.orientation

        self.get_logger().info(
            f"ABB PLACE GOAL -> Pos: [x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}] "
            f"| Orient: [x={o.x:.3f}, y={o.y:.3f}, z={o.z:.3f}, w={o.w:.3f}]"
        )

        action_result = await self.send_action_goal(self.move_client, abb_place_goal)
        self.get_logger().info(f'ABB Place Action Result: {action_result}')
        if action_result:
            self.get_logger().info(f'ABB Placed')
        
        return action_result

    # ---------- ACTION SERVER CALLBACK ----------
    async def execute_task_callback(self, goal_handle):
        goal = goal_handle.request
        task_type = goal.task_type
        target_pose = goal.target_pose

        self.get_logger().info(f"Received ExecuteTask: {task_type} for ABB")

        if task_type == "PICK":
            success = await self.execute_abb_pick(goal.target_pose, goal_handle)
        elif task_type == "PLACE":
            success = await self.execute_abb_place(goal.target_pose, goal_handle)
        else:
            success = False
            
        # Send result back to supervisor
        result = ExecuteTask.Result()
        result.success = bool(success)
        # result.error_message = "" if success else "Failed to execute ABB task"
        # goal_handle.succeed()
        # return result
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

# ---------- MAIN ----------
def main(args=None):
    rclpy.init(args=args)
    node = ABBController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
