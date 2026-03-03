#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

# Custom Interfaces
from supervisor_package.srv import GetAssemblyPlan
from dual_arms_msgs.msg import GraspPoint 
from dual_arms_msgs.srv import GetGrasp, DetectBricks
from geometry_msgs.msg import TransformStamped, Pose
from dual_arms_msgs.action import ExecuteTask

# TF2 Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class AssemblySupervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.cb_group = ReentrantCallbackGroup()
        
        # --- TF2 INITIALIZATION ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- CLIENTS ---
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.grasp_pipeline_client = self.create_client(GetGrasp, 'grasp/get_grasp_point', callback_group=self.cb_group) 
        
        # ONLY TWO ACTION CLIENTS NEEDED NOW!
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)
        self.ar4_task_client = ActionClient(self, ExecuteTask, 'ar4_control', callback_group=self.cb_group)
        
        self.get_logger().info('Supervisor Initialized. Waiting for services...')

        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []
        self.detected_bricks = []
        self.current_grasp_point = None

        self.timer = self.create_timer(1.0, self.state_machine_loop, callback_group=self.cb_group)

    def transform_pose_to_abb(self, input_pose:Pose):
        try:
            transformed_pose = Pose()
            transformed_pose.position.x = input_pose.position.y + 0.674
            transformed_pose.position.y = input_pose.position.x + 0.033
            transformed_pose.orientation.x = input_pose.orientation.x
            transformed_pose.orientation.y = input_pose.orientation.y
            transformed_pose.orientation.z = input_pose.orientation.z
            transformed_pose.orientation.w = input_pose.orientation.w
            return transformed_pose
        except Exception as ex:
            self.get_logger().error(f'TF2 Error: {ex}')
            return input_pose 

    async def state_machine_loop(self):
        self.timer.cancel()
        try:
            if self.state == "INIT":
                if not self.gui_client.wait_for_service(timeout_sec=1.0):
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

                req = GetAssemblyPlan.Request()
                result = await self.gui_client.call_async(req)
                
                if result is not None and len(result.plan) > 0:
                    self.assembly_queue = result.plan
                    self.state = "DETECT"
                else:
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

            elif self.state == "DETECT":
                if not self.camera_client.wait_for_service(timeout_sec=1.0):
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

                req = DetectBricks.Request()
                result = await self.camera_client.call_async(req)
                
                for brick in result.bricks:
                    brick.pose = self.transform_pose_to_abb(brick.pose)
                self.detected_bricks = result.bricks
                self.handover_pose = self.transform_pose_to_abb(result.handover_pose)
                self.state = "PROCESS_NEXT"

            elif self.state == "PROCESS_NEXT":
                if not self.assembly_queue:
                    self.get_logger().info('All tasks complete!')
                    self.state = "DONE"
                    return

                self.current_brick = self.assembly_queue.pop(0)
                self.state = "GRASP_PIPELINE"

            elif self.state == "GRASP_PIPELINE":
                if not self.grasp_pipeline_client.wait_for_service(timeout_sec=2.0):
                    self.state = "PROCESS_NEXT"
                    return

                grasp_req = GetGrasp.Request()
                grasp_req.brick_index = str(self.current_brick.id)
                grasp_result = await self.grasp_pipeline_client.call_async(grasp_req)

                if grasp_result.success:
                    raw_grasp = grasp_result.grasp_point
                    raw_grasp.pose = self.transform_pose_to_abb(raw_grasp.pose)
                    raw_grasp.pose.position.z = 0.22
                    self.current_grasp_point = raw_grasp

                    if self.current_brick.start_side == "ABB":
                        self.state = "EXECUTE_ABB_PICK"
                    elif self.current_brick.start_side == "HANDOVER":
                        self.state = "HANDOVER_SEQUENCE"
                    elif self.current_brick.start_side == "AR4":
                        self.state = "EXECUTE_AR4_DIRECT"
                else:
                    self.state = "PROCESS_NEXT"

            # =========================
            # AR4 EXECUTIONS (Now just 1 goal!)
            # =========================
            elif self.state in ["EXECUTE_AR4_DIRECT", "AR4_PICK_FOR_HANDOVER"]:
                self.get_logger().info(f'Starting AR4 Pick Sequence for {self.current_brick.id}')

                ar4_pick_goal = ExecuteTask.Goal()
                ar4_pick_goal.task_type = "PICK"

                current_x = -self.current_grasp_point.pose.orientation.z
                current_y = -self.current_grasp_point.pose.orientation.w
                # # The magic number 0.7071 is 1/sqrt(2)
                factor = 0.70710678

                # To rotate +90 degrees (Counter-Clockwise)
                new_x = factor * (current_x - current_y)
                new_y = factor * (current_x + current_y)

                # To rotate -90 degrees (Clockwise)
                new_x = factor * (current_x + current_y)
                new_y = factor * (current_y - current_x)

                # Assign back to your message
                self.current_grasp_point.pose.orientation.x = new_x
                self.current_grasp_point.pose.orientation.y = new_y
                self.current_grasp_point.pose.orientation.z = 0.0
                self.current_grasp_point.pose.orientation.w = 0.0

                # self.current_grasp_point.pose.orientation.x = -self.current_grasp_point.pose.orientation.z
                # self.current_grasp_point.pose.orientation.y = -self.current_grasp_point.pose.orientation.w
                # self.current_grasp_point.pose.orientation.z = 0.0
                # self.current_grasp_point.pose.orientation.w = 0.0
                self.current_grasp_point.pose.position.z = 0.14
                
                ar4_pick_goal.target_pose = self.current_grasp_point.pose 

                action_result = await self.send_action_goal(self.ar4_task_client, ar4_pick_goal)
                if await self.check_and_recover(action_result, self.state): 
                    return

                self.state = "AR4_PLACE_ON_GRID" if self.state == "EXECUTE_AR4_DIRECT" else "HANDOVER_EXECUTION"

            elif self.state == "AR4_PLACE_ON_GRID":
                self.get_logger().info(f'Starting AR4 Place Sequence')
                ar4_place_goal = ExecuteTask.Goal()
                ar4_place_goal.task_type = "PLACE"

                self.current_brick.place_pose.orientation.x = -self.current_brick.place_pose.orientation.z
                self.current_brick.place_pose.orientation.y = -self.current_brick.place_pose.orientation.w
                self.current_brick.place_pose.orientation.z = 0.0
                self.current_brick.place_pose.orientation.w = 0.0
                self.current_brick.place_pose.position.z = 0.14

                ar4_place_goal.target_pose = self.current_brick.place_pose 
                
                action_result = await self.send_action_goal(self.ar4_task_client, ar4_place_goal)
                if await self.check_and_recover(action_result, self.state):
                    return
                self.state = "PROCESS_NEXT"

            # =========================
            # ABB EXECUTIONS
            # =========================
            elif self.state == "EXECUTE_ABB_PICK":
                self.get_logger().info(f'Starting ABB Pick for {self.current_brick.id}')
                abb_pick_goal = ExecuteTask.Goal()
                abb_pick_goal.task_type = "PICK"
                
                current_x = -self.current_grasp_point.pose.orientation.z
                current_y = -self.current_grasp_point.pose.orientation.w
              
                
                # # The magic number 0.7071 is 1/sqrt(2)
                # factor = 0.70710678

                # # To rotate +90 degrees (Counter-Clockwise)
                # new_x = factor * (current_x - current_y)
                # new_y = factor * (current_x + current_y)

                # To rotate -90 degrees (Clockwise)
                # new_x = factor * (current_x + current_y)
                # new_y = factor * (current_y - current_x)

                # Assign back to your message
                # self.current_grasp_point.pose.orientation.x = new_x
                # self.current_grasp_point.pose.orientation.y = new_y
                # self.current_grasp_point.pose.orientation.z = 0.0
                # self.current_grasp_point.pose.orientation.w = 0.0

                self.current_grasp_point.pose.orientation.x = current_x
                self.current_grasp_point.pose.orientation.y = current_y
                self.current_grasp_point.pose.orientation.z = 0.0
                self.current_grasp_point.pose.orientation.w = 0.0

                self.current_grasp_point.pose.position.z = 0.21


                if self.current_grasp_point:
                    abb_pick_goal.target_pose = self.current_grasp_point.pose
                else:
                    abb_pick_goal.target_pose = self.current_brick.pickup_pose 
                
                action_result = await self.send_action_goal(self.abb_client, abb_pick_goal)
                if await self.check_and_recover(action_result, self.state): return
                self.state = "EXECUTE_ABB_PLACE"
                
            elif self.state == "EXECUTE_ABB_PLACE":
                abb_place_goal = ExecuteTask.Goal()
                abb_place_goal.task_type = "PLACE"
                abb_place_goal.target_pose = self.current_brick.place_pose 
                abb_place_goal.target_pose.position.z = 0.24

                current_x = -self.current_brick.place_pose.orientation.z
                current_y = -self.current_brick.place_pose.orientation.w
              

                self.current_brick.place_pose.orientation.x = current_x
                self.current_brick.place_pose.orientation.y = current_y
                self.current_brick.place_pose.orientation.z = 0.0
                self.current_brick.place_pose.orientation.w = 0.0

                
                action_result = await self.send_action_goal(self.abb_client, abb_place_goal)
                if await self.check_and_recover(action_result, self.state): return
                self.state = "PROCESS_NEXT"
                
            # =========================
            # HANDOVER
            # =========================
            elif self.state == "HANDOVER_SEQUENCE":
                self.state = "AR4_PICK_FOR_HANDOVER"

            elif self.state == "HANDOVER_EXECUTION":
                self.get_logger().info('Executing Handover is pending refactor based on new task servers...')
                # Since Handover requires a coordinated sequence between BOTH servers, 
                # we will need to define a "GOTO_HANDOVER" task_type in the future.
                self.state = "PROCESS_NEXT"

            elif self.state == "RECOVERY":
                self.get_logger().warn('Recovery Mode triggered. Operator intervention required.')
                self.state = "EMERGENCY_STOP"

            elif self.state == "EMERGENCY_STOP":
                return 

        except Exception as e:
            self.get_logger().error(f'State Machine Failed: {e}')

        if self.state != "DONE":
            self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)

    async def send_action_goal(self, client, goal_msg):
        if not client.wait_for_server(timeout_sec=5.0):
            return None
        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            return None
        result_future = goal_handle.get_result_async()
        result = await result_future
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return result.result
        return None
        
    async def check_and_recover(self, result, current_state_name):
        if result is None or not result.success:
            self.get_logger().error(f"Failure in {current_state_name}. Moving to RECOVERY.")
            self.state = "RECOVERY"
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = AssemblySupervisor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()