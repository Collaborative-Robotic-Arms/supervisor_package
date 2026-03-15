#!/usr/bin/env python3
"""
Hybrid Supervisor with MTC Integration

This enhanced supervisor adds:
1. Zone detection for handover areas
2. Dynamic switching between multithreaded and MTC control modes
3. MTC-based collaborative handover execution
4. True Parallel Execution using native rclpy timers
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_geometry_msgs
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Pose
# Custom Interfaces
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.action import MoveToPose, AlignToTarget
from dual_arms_msgs.msg import GraspPoint 
from dual_arms_msgs.srv import GetGrasp, DetectBricks, GetHandoverZone, ExecuteMTCHandover, ResolveCollision
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from dual_arms_msgs.action import ExecuteTask
from std_srvs.srv import SetBool

from tf2_ros import TransformException, Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String


class HybridAssemblySupervisor(Node):
    """
    Enhanced supervisor that switches between multithreaded and MTC control
    based on proximity to handover zones.
    """
    
    def __init__(self):
        super().__init__('hybrid_supervisor')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # --- PARAMETERS ---
        self.declare_parameter('use_sim', True)
        self.declare_parameter('enable_mtc_mode', True)
        self.declare_parameter('handover_trigger_distance', 0.3)  # Distance to trigger MTC
        
        use_sim_value = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.enable_mtc_mode = self.get_parameter('enable_mtc_mode').get_parameter_value().bool_value
        self.handover_trigger_distance = self.get_parameter('handover_trigger_distance').get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- SERVICE CLIENTS ---
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.grasp_pipeline_client = self.create_client(GetGrasp, 'grasp/get_grasp_point', callback_group=self.cb_group)
        self.gripper_client = self.create_client(SetBool, 'ar4_gripper/set', callback_group=self.cb_group)
        self.abb_gripper_client = self.create_client(SetBool, 'abb_gripper/set', callback_group=self.cb_group) 
        # MTC-specific service
        self.zone_client = self.create_client(GetHandoverZone, 'zone_detection/get_handover_zone', callback_group=self.cb_group)
        self.mtc_handover_client = self.create_client(ExecuteMTCHandover, 'mtc_controller/execute_handover', callback_group=self.cb_group)
        self.mtc_resolve_client = self.create_client(ResolveCollision, 'mtc_controller/resolve_collision', callback_group=self.cb_group)

        # Track active targets for MTC recovery
        self.ar4_active_target = None
        self.abb_active_target = None

        # --- ACTION CLIENTS ---
        self.ar4_client = ActionClient(self, ExecuteTask, 'ar4_control', callback_group=self.cb_group)
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        # --- INTERNAL STATE ---
        self.mtc_active = False
        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []
        self.detected_bricks = []
        self.current_grasp_point = None
        self.handover_pose = None
        self.ar4_busy = False
        self.abb_busy = False
        
        # Handover-specific state
        self.operation_type = None
        self.intermediate_pose = None
        self.abb_grasp_point_for_handover = None
        self.ar4_current_pose = None
        self.abb_current_pose = None
        
        # Parallel Execution Trackers
        self.ar4_parallel_done = False
        self.abb_parallel_done = False
        self.ar4_timer = None
        self.abb_timer = None

        # --- STAGE AND EXECUTION TRACKING (Update 1) ---
        self.ar4_stage = "IDLE"
        self.abb_stage = "IDLE"
        self.ar4_cancelled = False
        self.abb_cancelled = False
        self.ar4_goal_handle = None
        self.abb_goal_handle = None
        self.ar4_current_brick = None
        self.abb_current_brick = None
        self.ar4_recovering = False
        self.abb_recovering = False

        # MTC state tracking
        self.control_mode = "MULTITHREADED"  # MULTITHREADED or MTC_HANDOVER
        self.in_handover_zone = False
        self.mtc_task_id = None

        # --- E-STOP CONFIG ---
        self.emergency_stop = False
        self.zone_sub = self.create_subscription(
            String,
            '/zone_status',
            self.zone_status_callback,
            10,
            callback_group=self.cb_group
        )

        self.get_logger().info('Hybrid Assembly Supervisor Initialized')
        self.timer = self.create_timer(1.0, self.state_machine_loop, callback_group=self.cb_group)

    # ========================================================================
    # ZONE DETECTION AND MODE SWITCHING
    # ========================================================================

    def detect_handover_proximity(self, pose):
        """Check if pose is approaching handover zone"""
        if self.handover_pose is None:
            return False
            
        dx = pose.position.x - self.handover_pose.position.x
        dy = pose.position.y - self.handover_pose.position.y
        dz = pose.position.z - self.handover_pose.position.z
        
        distance = (dx**2 + dy**2 + dz**2)**0.5
        return distance <= self.handover_trigger_distance
    
    def trigger_mtc_resolution(self):
        self.mtc_active = True
        
        # 1. Kill the independent Task Servers so they stop sending commands
        self.stop_parallel_workers() 

        # 2. Prepare the goal with the FINAL place poses from your brick data
        goal_msg = ExecuteTask.Goal()
        goal_msg.ar4_target = self.current_brick.ar4_place_pose
        goal_msg.abb_target = self.current_brick.abb_place_pose

        # 3. Send the goal and wait for the RESULT callback
        self.get_logger().info("MTC taking over the full 12-DOF path to the targets...")
        self.mtc_client.send_goal_async(goal_msg).add_done_callback(self.mtc_result_callback)

    def mtc_result_callback(self, future):
        # Once MTC reports success, reset the flag and move to the next task
        self.get_logger().info("✅ MTC Resolution Complete. Poses reached.")
        self.mtc_active = False
        self.state = "PROCESS_NEXT_BRICK"

    def stop_parallel_workers(self):
        """Cleanly halts the async timers and clears busy flags for a fresh start."""
        if self.ar4_timer:
            self.ar4_timer.cancel()
            self.ar4_timer = None
        if self.abb_timer:
            self.abb_timer.cancel()
            self.abb_timer = None
        
        # Reset the logic locks so they can be re-dispatched safely
        self.ar4_busy = False
        self.abb_busy = False
        self.ar4_parallel_done = False
        self.abb_parallel_done = False

    async def switch_control_mode(self, new_mode):
        """Switch between multithreaded and MTC control modes"""
        if self.control_mode != new_mode:
            old_mode = self.control_mode
            self.control_mode = new_mode
            self.get_logger().warn(f'Control mode switched: {old_mode} -> {new_mode}')
            
            if new_mode == "MTC_HANDOVER":
                self.get_logger().info('MTC handover mode activated for collaborative task')
            else:
                self.get_logger().info('Switched back to standard multithreaded control')

    # ========================================================================
    # HELPER METHODS FOR HANDOVER DECISIONS
    # ========================================================================
    
    def is_handover_operation(self, brick):
        if brick.start_side == brick.target_side:
            return False
        if brick.start_side not in ["AR4", "ABB"] or brick.target_side not in ["AR4", "ABB"]:
            return False
        return True

    def calculate_intermediate_pose(self, ar4_position, abb_target, height_offset=0.1):
        intermediate = Pose()
        intermediate.position.x = (ar4_position.position.x * 0.3 + abb_target.position.x * 0.7)
        intermediate.position.y = (ar4_position.position.y + abb_target.position.y) / 2.0
        intermediate.position.z = max(ar4_position.position.z, abb_target.position.z) + height_offset
        intermediate.orientation = ar4_position.orientation
        
        self.get_logger().info(
            f'Intermediate pose calculated: x={intermediate.position.x:.3f}, '
            f'y={intermediate.position.y:.3f}, z={intermediate.position.z:.3f}'
        )
        return intermediate

    def transform_stamped_to_pose(self, transform_stamped):
        pose = Pose()
        pose.position.x = transform_stamped.transform.translation.x
        pose.position.y = transform_stamped.transform.translation.y
        pose.position.z = transform_stamped.transform.translation.z
        pose.orientation = transform_stamped.transform.rotation
        return pose

    async def get_current_arm_poses(self):
        try:
            t_ar4 = self.tf_buffer.lookup_transform('abb_table', 'ar4_ee_link', rclpy.time.Time())
            self.ar4_current_pose = self.transform_stamped_to_pose(t_ar4)

            t_abb = self.tf_buffer.lookup_transform('abb_table', 'tool0', rclpy.time.Time())
            self.abb_current_pose = self.transform_stamped_to_pose(t_abb)
            return True
        except Exception as e:
            self.get_logger().warn(f'Could not get arm poses: {e}')
            return False

    # ========================================================================
    # PARALLEL EXECUTION HELPER SEQUENCES
    # ========================================================================

    async def execute_ar4_full_sequence(self, brick):
        self.get_logger().info(f'[PARALLEL] AR4 starting sequence for brick {brick.id}')
        
        req = GetGrasp.Request()
        req.brick_index = str(brick.id)
        res = await self.grasp_pipeline_client.call_async(req)
        if not res.success: return False
        if self.emergency_stop or self.ar4_cancelled: return False

        grasp = res.grasp_point
        # grasp.pose = self.transform_pose_to_abb(grasp.pose)
        grasp.pose.position.z = 0.22 
        
        # --- UPDATE STAGE BEFORE GOAL (Update 2) ---
        self.ar4_stage = "PICK"
        pick_goal = ExecuteTask.Goal(task_type="PICK", target_pose=grasp.pose)
        result = await self.send_action_goal(self.ar4_client, pick_goal)
        
        # --- STOP WORKERS IF CANCEL HAPPENS (Update 5) ---
        if self.ar4_cancelled:
            self.get_logger().warn("AR4 execution cancelled by supervisor")
            return False
            
        if result is None: return False

        if self.emergency_stop or self.ar4_cancelled: return False
        # --- UPDATE STAGE BEFORE GOAL (Update 2) ---
        self.ar4_stage = "PLACE"
        plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=brick.place_pose)
        plc_goal.target_pose.position.z = 0.26 
        result = await self.send_action_goal(self.ar4_client, plc_goal)
        
        # --- STOP WORKERS IF CANCEL HAPPENS (Update 5) ---
        if self.ar4_cancelled:
            self.get_logger().warn("AR4 execution cancelled by supervisor")
            return False
            
        if result is None: return False

        # --- UPDATE STAGE ON SUCCESS (Update 2) ---
        self.ar4_stage = "DONE"
        self.get_logger().info(f'[PARALLEL] AR4 finished brick {brick.id}')
        return True
    
    async def execute_abb_full_sequence(self, brick):
        self.get_logger().info(f'[PARALLEL] ABB starting sequence for brick {brick.id}')
        if self.emergency_stop or self.abb_cancelled: return False

        # --- UPDATE STAGE BEFORE GOAL (Update 2) ---
        self.abb_stage = "PICK"
        pick_goal = ExecuteTask.Goal()
        pick_goal.task_type = "PICK"
        pick_goal.target_pose = brick.pickup_pose 
        result = await self.send_action_goal(self.abb_client, pick_goal)
        
        # --- STOP WORKERS IF CANCEL HAPPENS (Update 5) ---
        if self.abb_cancelled:
            self.get_logger().warn("ABB execution cancelled by supervisor")
            return False
            
        if result is None: return False
        
        # --- UPDATE STAGE BEFORE GOAL (Update 2) ---
        self.abb_stage = "PLACE"
        plc_goal = ExecuteTask.Goal()
        plc_goal.task_type = "PLACE"
        plc_goal.target_pose = brick.place_pose
        plc_goal.target_pose.position.z = 0.24 
        result = await self.send_action_goal(self.abb_client, plc_goal)
        
        # --- STOP WORKERS IF CANCEL HAPPENS (Update 5) ---
        if self.abb_cancelled:
            self.get_logger().warn("ABB execution cancelled by supervisor")
            return False
            
        if result is None: return False
        
        # --- UPDATE STAGE ON SUCCESS (Update 2) ---
        self.abb_stage = "DONE"
        self.get_logger().info(f'[PARALLEL] ABB finished brick {brick.id}')
        return True

    async def execute_ar4_worker(self, brick):
        self.get_logger().info(f'[AR4 WORKER] Starting sequence for brick {brick.id}')
        
        req = GetGrasp.Request()
        req.brick_index = str(brick.id)
        res = await self.grasp_pipeline_client.call_async(req)
        if not res or not res.success:
            self.ar4_busy = False
            return

        if self.emergency_stop or self.ar4_cancelled: 
            self.ar4_busy = False
            return
        
        grasp = res.grasp_point
        # grasp.pose = self.transform_pose_to_abb(grasp.pose)
        grasp.pose.position.z = 0.22 
        
        # --- UPDATE STAGE BEFORE GOAL (Update 2) ---
        self.ar4_stage = "PICK"
        pick_goal = ExecuteTask.Goal(task_type="PICK", target_pose=grasp.pose)
        result = await self.send_action_goal(self.ar4_client, pick_goal)
        
        # --- STOP WORKERS IF CANCEL HAPPENS (Update 5) ---
        if self.ar4_cancelled:
            self.get_logger().warn("AR4 execution cancelled by supervisor")
            return
            

        if result is None or self.ar4_cancelled: 
            self.ar4_busy = False
            return
        
        # --- UPDATE STAGE BEFORE GOAL (Update 2) ---
        self.ar4_stage = "PLACE"
        plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=brick.place_pose)
        plc_goal.target_pose.position.z = 0.22 
        result = await self.send_action_goal(self.ar4_client, plc_goal)
        
        # --- STOP WORKERS IF CANCEL HAPPENS (Update 5) ---
        if self.ar4_cancelled:
            self.get_logger().warn("AR4 execution cancelled by supervisor")
            return

        # --- UPDATE STAGE ON SUCCESS (Update 2) ---
        self.ar4_stage = "DONE"
        self.ar4_busy = False 
        self.get_logger().info(f'[AR4 WORKER] Task complete and arm homed.')

    async def execute_abb_worker(self, brick):
        self.get_logger().info(f'[ABB WORKER] Starting sequence for brick {brick.id}')
        if self.emergency_stop or self.abb_cancelled:
            self.abb_busy = False
            return
            
        self.abb_stage = "PICK"
        result = await self.send_action_goal(self.abb_client, ExecuteTask.Goal(task_type="PICK", target_pose=brick.pickup_pose))
        
        # --- FIX: RELEASE LOCK ON DEATH ---
        if result is None or self.abb_cancelled:
            self.get_logger().warn("ABB execution cancelled or failed")
            self.abb_busy = False # <-- Make sure this is here!
            return
        
        self.abb_stage = "PLACE"
        plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=brick.place_pose)
        plc_goal.target_pose.position.z = 0.24 
        result = await self.send_action_goal(self.abb_client, plc_goal)
        
        # --- FIX: RELEASE LOCK ON DEATH ---
        if result is None or self.emergency_stop or self.abb_cancelled:
            self.get_logger().warn("ABB execution cancelled or failed")
            self.abb_busy = False # <-- Make sure this is here!
            return
        
        self.abb_stage = "DONE"
        self.abb_busy = False 
        self.get_logger().info(f'[ABB WORKER] Task complete and arm homed.')

    # ========================================================================
    # SUPERVISOR STATE MACHINE WITH MTC INTEGRATION
    # ========================================================================

    def zone_status_callback(self, msg):
        if "COLLISION_WARNING" in msg.data and not self.emergency_stop and not self.mtc_active and not self.ar4_recovering and not self.abb_recovering:
            # NEW: Log exact state upon entry
            self.get_logger().warn(f"⚠️ PROXIMITY ALERT! Interrupting AR4 in [{self.ar4_stage}] and ABB in [{self.abb_stage}]")
            
            self.mtc_active = True
            self.emergency_stop = True
            self.ar4_cancelled = True
            self.abb_cancelled = True
            
            if self.ar4_goal_handle: self.ar4_goal_handle.cancel_goal_async()
            if self.abb_goal_handle: self.abb_goal_handle.cancel_goal_async()

            self.state = "TRIGGER_MTC_SAFE_RESOLUTION"

    async def state_machine_loop(self):
        self.timer.cancel()
        try:
            if self.state == "INIT":
                self.get_logger().info('Requesting Assembly Plan from GUI...')
                
                if not self.gui_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for GUI node...')
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

                req = GetAssemblyPlan.Request()
                result = await self.gui_client.call_async(req)
                
                if result is not None and len(result.plan) > 0:
                    self.assembly_queue = result.plan
                    self.get_logger().info(f'Plan received! {len(self.assembly_queue)} bricks to process.')
                    self.state = "DETECT"
                else:
                    self.get_logger().warn('Assembly plan is empty. Retrying...')
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

            elif self.state == "DETECT":
                self.get_logger().info('Requesting Camera Detection...')
                
                if not self.camera_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().error('Camera Detection service not available!')
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

                req = DetectBricks.Request()
                result = await self.camera_client.call_async(req)
                
                # for brick in result.bricks:
                #     brick.pose = self.transform_pose_to_abb(brick.pose)
                
                self.detected_bricks = result.bricks
                # self.handover_pose = self.transform_pose_to_abb(result.handover_pose)
                
                self.state = "PROCESS_NEXT"

            elif self.state == "PROCESS_NEXT":
                if not self.assembly_queue and not self.ar4_busy and not self.abb_busy:
                    self.get_logger().info('✅ All assembly tasks complete!')
                    self.state = "DONE"
                    return

                # --- AR4 DISPATCHER ---
                if not self.ar4_busy and self.state != "AR4_PLACE_ON_GRID":
                    ar4_brick = next((b for b in self.assembly_queue if b.start_side == "AR4"), None)
                    if ar4_brick:
                        self.ar4_current_brick = ar4_brick
                        if self.is_handover_operation(ar4_brick):
                            self.assembly_queue.remove(ar4_brick)
                            self.current_brick = ar4_brick
                            self.state = "GRASP_PIPELINE"
                            return
                        else:
                            self.assembly_queue.remove(ar4_brick)
                            self.ar4_busy = True
                            # Use a non-lambda wrapper to ensure it is awaited
                            def ar4_callback():
                                if self.ar4_timer:
                                    self.ar4_timer.cancel()
                                self.executor.create_task(self.execute_ar4_worker(ar4_brick))
                            self.ar4_timer = self.create_timer(0.01, ar4_callback, callback_group=self.cb_group)

                # --- DISPATCH TO ABB ---
                if not self.abb_busy and self.state != "EXECUTE_ABB_PLACE":
                    abb_brick = next((b for b in self.assembly_queue if b.start_side == "ABB"), None)
                    if abb_brick:
                        self.abb_current_brick = abb_brick
                        if self.is_handover_operation(abb_brick):
                            if not self.ar4_busy:
                                self.assembly_queue.remove(abb_brick)
                                self.current_brick = abb_brick
                                self.state = "GRASP_PIPELINE"
                                return
                        else:
                            self.assembly_queue.remove(abb_brick)
                            self.abb_busy = True
                            # Use a non-lambda wrapper to ensure it is awaited
                            def abb_callback():
                                if self.abb_timer:
                                    self.abb_timer.cancel()
                                self.executor.create_task(self.execute_abb_worker(abb_brick))
                            self.abb_timer = self.create_timer(0.01, abb_callback, callback_group=self.cb_group)

            elif self.state == "GRASP_PIPELINE":
                self.get_logger().info(f'Getting grasp for Brick {self.current_brick.id}')

                if not self.grasp_pipeline_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('Grasp Pipeline service not available!')
                    self.state = "PROCESS_NEXT"
                    return

                grasp_req = GetGrasp.Request()
                grasp_req.brick_index = str(self.current_brick.id)
                grasp_result = await self.grasp_pipeline_client.call_async(grasp_req)

                if grasp_result.success:
                    raw_grasp = grasp_result.grasp_point
                    # raw_grasp.pose = self.transform_pose_to_abb(raw_grasp.pose)
                    raw_grasp.pose.position.z = 0.22
                    self.current_grasp_point = raw_grasp
                    
                    self.get_logger().info(f'Grasp retrieved. Quality: {self.current_grasp_point.quality}')
                    
                    if self.is_handover_operation(self.current_brick):
                        self.get_logger().info(
                            f'🔄 HANDOVER detected: {self.current_brick.start_side} → {self.current_brick.target_side}'
                        )
                        self.operation_type = "HANDOVER"
                        self.state = "AR4_PICK_FOR_HANDOVER"
                        
                    elif len(self.assembly_queue) > 0:
                        self.get_logger().info(f'⚡ Multiple bricks detected - checking for parallel execution')
                        
                        ar4_bricks = [b for b in self.assembly_queue if b.start_side == "AR4"]
                        abb_bricks = [b for b in self.assembly_queue if b.start_side == "ABB"]
                        
                        if ar4_bricks and abb_bricks:
                            self.operation_type = "PARALLEL"
                            self.state = "INITIALIZE_PARALLEL_EXECUTION"
                        else:
                            self.operation_type = "SEQUENTIAL"
                            self.state = self.current_brick.start_side  
                    else:
                        self.get_logger().info(f'📦 Sequential pick/place operation')
                        self.operation_type = "SEQUENTIAL"
                        self.state = self.current_brick.start_side  
                else:
                    self.get_logger().error(f'Failed to get grasp for brick {self.current_brick.id}')
                    self.state = "PROCESS_NEXT"

            # ================================================================
            # PARALLEL EXECUTION STATE (ROS 2 NATIVE FIX)
            # ================================================================
            elif self.state == "INITIALIZE_PARALLEL_EXECUTION":
                self.get_logger().info('🔄 Initializing TRUE parallel execution...')
                
                brick1 = self.current_brick
                partner_side = "ABB" if brick1.start_side == "AR4" else "AR4"
                brick2 = next((b for b in self.assembly_queue if b.start_side == partner_side), None)
                
                if brick1 and brick2:
                    self.assembly_queue.remove(brick2) 
                    
                    ar4_brick = brick1 if brick1.start_side == "AR4" else brick2
                    abb_brick = brick1 if brick1.start_side == "ABB" else brick2
                    
                    self.get_logger().info(f'⚡ TRUE PARALLEL: AR4 -> Brick {ar4_brick.id}, ABB -> Brick {abb_brick.id}')
                    
                    # Reset Done Flags and Lock the workers
                    self.ar4_parallel_done = False
                    self.abb_parallel_done = False
                    self.ar4_busy = True  # <--- Lock AR4
                    self.abb_busy = True  # <--- Lock ABB

                    # Create wrappers to handle the async execution and set flags
                    async def run_ar4():
                        self.ar4_timer.cancel()
                        await self.execute_ar4_full_sequence(ar4_brick)
                        self.ar4_parallel_done = True
                        self.ar4_busy = False # <--- Safely release lock when fully finished
                        
                    async def run_abb():
                        self.abb_timer.cancel()
                        await self.execute_abb_full_sequence(abb_brick)
                        self.abb_parallel_done = True
                        self.abb_busy = False # <--- Safely release lock when fully finished
                        
                    # Spawn both tasks instantly as ROS 2 timers
                    self.ar4_timer = self.create_timer(0.01, run_ar4, callback_group=self.cb_group)
                    self.abb_timer = self.create_timer(0.01, run_abb, callback_group=self.cb_group)
                    
                    # Transition to a waiting state
                    self.state = "WAIT_FOR_PARALLEL"
                else:
                    self.get_logger().warn('Partner brick not found. Falling back to sequential.')
                    if self.current_brick.start_side == "AR4":
                        self.state = "EXECUTE_AR4_DIRECT"
                    else:
                        self.state = "EXECUTE_ABB_PICK"

            elif self.state == "WAIT_FOR_PARALLEL":
                # Constantly check if both arms have finished their tasks
                if self.ar4_parallel_done and self.abb_parallel_done:
                    self.get_logger().info('✅ TRUE PARALLEL EXECUTION COMPLETE!')
                    self.state = "PROCESS_NEXT"

            # ================================================================
            # MTC-BASED HANDOVER STATE
            # ================================================================
            elif self.state == "MTC_HANDOVER_EXECUTION":
                self.get_logger().info('Executing MTC-based collaborative handover...')

                if not self.mtc_handover_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('MTC handover service not available! Falling back to standard mode.')
                    await self.switch_control_mode("MULTITHREADED")
                    self.state = "HANDOVER_SEQUENCE"
                    return

                mtc_req = ExecuteMTCHandover.Request()
                mtc_req.ar4_start_pose = self.current_grasp_point.pose
                mtc_req.abb_start_pose = self.current_brick.pickup_pose
                mtc_req.handover_pose = self.handover_pose
                mtc_req.object_id = self.current_brick.type

                mtc_result = await self.mtc_handover_client.call_async(mtc_req)

                if mtc_result.success:
                    self.get_logger().info(f'MTC handover completed. Execution ID: {mtc_result.execution_id}')
                    self.mtc_task_id = mtc_result.execution_id
                    self.state = "PROCESS_NEXT"
                else:
                    self.get_logger().error(f'MTC handover failed: {mtc_result.status_message}')
                    await self.switch_control_mode("MULTITHREADED")
                    self.state = "HANDOVER_SEQUENCE"

            # ================================================================
            # SEQUENTIAL FALLBACK STATES
            # ================================================================

            elif self.state == "EXECUTE_AR4_DIRECT":
                self.get_logger().info(f'AR4 Direct Pick for {self.current_brick.id}')
                self.ar4_stage = "PICK"
                
                await self.set_ar4_gripper(True)

                goal_msg = MoveToPose.Goal()
                goal_msg.target_pose = self.current_grasp_point.pose
                goal_msg.strategy = "APPROACH_OFFSET"
                action_result = await self.send_action_goal(self.ar4_client, goal_msg)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                await self.set_ar4_gripper(False)

                grasp_goal = MoveToPose.Goal()
                grasp_goal.target_pose = self.current_grasp_point.pose
                grasp_goal.strategy = "GRASP"
                action_result = await self.send_action_goal(self.ar4_client, grasp_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return

                self.state = "AR4_PLACE_ON_GRID"

            elif self.state == "AR4_PLACE_ON_GRID":
                self.ar4_stage = "PLACE"
                place_goal = MoveToPose.Goal()
                place_goal.target_pose = self.current_brick.place_pose
                place_goal.target_pose.position.z = 0.22 
                place_goal.strategy = "PLACE"
                action_result = await self.send_action_goal(self.ar4_client, place_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return
                
                self.get_logger().info('Retracting AR4 to safe position...')
                retract_goal = MoveToPose.Goal()
                retract_goal.strategy = "HOME"
                await self.send_action_goal(self.ar4_client, retract_goal)

                self.ar4_stage = "DONE"
                self.state = "PROCESS_NEXT"

            elif self.state == "EXECUTE_ABB_PICK":
                self.get_logger().info(f'ABB Pick for {self.current_brick.id}')
                self.abb_stage = "PICK"

                abb_pick_goal = ExecuteTask.Goal()
                abb_pick_goal.task_type = "PICK"
                abb_pick_goal.target_pose = self.current_grasp_point.pose if self.current_grasp_point else self.current_brick.pickup_pose

                action_result = await self.send_action_goal(self.abb_client, abb_pick_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return
                self.state = "EXECUTE_ABB_PLACE"

            elif self.state == "EXECUTE_ABB_PLACE":
                self.abb_stage = "PLACE"
                abb_place_goal = ExecuteTask.Goal()
                abb_place_goal.task_type = "PLACE"
                abb_place_goal.target_pose = self.current_brick.place_pose
                abb_place_goal.target_pose.position.z = 0.24

                action_result = await self.send_action_goal(self.abb_client, abb_place_goal)
                if not action_result or not action_result.success:
                    self.state = "RECOVERY"
                    return
                
                self.abb_stage = "DONE"
                self.state = "PROCESS_NEXT"

            elif self.state == "RECOVERY":
                self.get_logger().warn('Entering Recovery Mode')
                recovery_goal = MoveToPose.Goal()
                recovery_goal.strategy = "HOME"
                action_result = await self.send_action_goal(self.ar4_client, recovery_goal)

                if action_result and action_result.success:
                    self.get_logger().info('Recovery successful')
                    self.assembly_queue.insert(0, self.current_brick)
                    self.state = "PROCESS_NEXT"
                else:
                    self.get_logger().error("Recovery failed! Emergency stop.")
                    self.state = "EMERGENCY_STOP"

            elif self.state == "TRIGGER_MTC_SAFE_RESOLUTION":
                self.get_logger().info('Engaging MTC to resolve proximity conflict safely...')
                
                if not self.mtc_resolve_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('MTC Resolution service missing! Halting safely.')
                    self.state = "EMERGENCY_STOP"
                    return
                
                await self.get_current_arm_poses()
                
                req = ResolveCollision.Request()
                req.ar4_target_pose = self.ar4_active_target if self.ar4_active_target else self.ar4_current_pose
                req.abb_target_pose = self.abb_active_target if self.abb_active_target else self.abb_current_pose
                
                result = await self.mtc_resolve_client.call_async(req)
                
                if result and result.success:
                    self.get_logger().info('✅ MTC Resolution Complete. Waiting for physical clearance...')
                    
                    # 1. Wait for physical clearance to prevent proximity sensor loop
                    await self.ros_sleep(1.5)
                    self.get_logger().info(f"🔄 EXITING MTC. AR4 was in [{self.ar4_stage}], ABB was in [{self.abb_stage}]")

                    # 2. Reset the system locks
                    self.mtc_active = False
                    self.emergency_stop = False
                    self.ar4_active_target = None
                    self.abb_active_target = None
                    self.ar4_cancelled = False
                    self.abb_cancelled = False

                    # 3. USE THE RECOVERY WORKERS INSTEAD OF LEGACY STATES
                    if self.ar4_stage in ["PICK", "PLACE"] and self.ar4_current_brick:
                        self.get_logger().info("Spawning AR4 Recovery Worker...")
                        self.ar4_busy = True
                        self.ar4_recovering = True
                        self.executor.create_task(self.recover_ar4_worker(self.ar4_stage, self.ar4_current_brick))
                    else:
                        self.ar4_busy = False

                    if self.abb_stage in ["PICK", "PLACE"] and self.abb_current_brick:
                        self.get_logger().info("Spawning ABB Recovery Worker...")
                        self.abb_busy = True
                        self.abb_recovering = True
                        self.executor.create_task(self.recover_abb_worker(self.abb_stage, self.abb_current_brick))
                    else:
                        self.abb_busy = False

                    # 4. Return to normal dispatching
                    self.state = "PROCESS_NEXT"
                else:
                    self.get_logger().error('MTC Resolution Failed.')
                    self.state = "EMERGENCY_STOP"

            elif self.state == "EMERGENCY_STOP":
                self.get_logger().error("SYSTEM HALTED - Manual Reset Required")
                await self.ros_sleep(3.0)
                self.state = "TRIGGER_MTC_SAFE_RESOLUTION"

        except Exception as e:
            self.get_logger().error(f'State machine error: {e}')

        if self.state != "DONE":
            self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)

    # ========================================================================
    # HELPER METHODS
    # ========================================================================
    async def ros_sleep(self, duration):
            """Non-blocking sleep for ROS 2 MultiThreadedExecutors"""
            future = rclpy.task.Future()
            timer = self.create_timer(
                duration, 
                lambda: future.set_result(None) if not future.done() else None, 
                callback_group=self.cb_group
            )
            await future
            self.destroy_timer(timer)
            
    def transform_pose_to_abb(self, input_pose):
        """Transform pose from camera frame to ABB base frame"""
        try:
            t = self.tf_buffer.lookup_transform('abb_table', 'camera_color_optical_frame', rclpy.time.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, t)
            return transformed_pose
        except TransformException as ex:
            return input_pose

    async def send_action_goal(self, client, goal_msg):
        """Send action goal and wait for result"""
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action server not available')
            return None

        # --- STORE THE ACTION GOAL HANDLES (Update 3) ---
        goal_future = client.send_goal_async(goal_msg)
        goal_handle = await goal_future

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected')
            return None

        # --- STORE GOAL HANDLE AND TARGET FOR MTC RECOVERY ---
        if client == self.ar4_client:  
            self.ar4_goal_handle = goal_handle
            if hasattr(goal_msg, 'target_pose'):
                self.ar4_active_target = goal_msg.target_pose
                
        elif client == self.abb_client:
            self.abb_goal_handle = goal_handle
            if hasattr(goal_msg, 'target_pose'):
                self.abb_active_target = goal_msg.target_pose

        result_future = goal_handle.get_result_async()
        result = await result_future

        # --- CLEAR GOAL HANDLE ONCE FINISHED ---
        if goal_handle is not None:
            if client == self.ar4_client and self.ar4_goal_handle is not None:
                if self.ar4_goal_handle == goal_handle:
                    self.ar4_goal_handle = None
            elif client == self.abb_client and self.abb_goal_handle is not None:
                if self.abb_goal_handle == goal_handle:
                    self.abb_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return result.result
        else:
            # FIX: If canceled by MTC, we no longer pause. We ABORT the thread.
            self.get_logger().warn(f'Action aborted or canceled. Original thread dying.')
            return None
            
    async def set_ar4_gripper(self, open_gripper: bool):
        """Control AR4 gripper"""
        if not self.gripper_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Gripper service not available')
            return False

        req = SetBool.Request()
        req.data = open_gripper
        result = await self.gripper_client.call_async(req)
        return result.success
    
    async def set_abb_gripper(self, open_gripper: bool):
        """Control ABB gripper"""
        if not self.abb_gripper_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('ABB Gripper service not available')
            return False
        req = SetBool.Request()
        req.data = open_gripper
        result = await self.abb_gripper_client.call_async(req)
        return result.success
    
    async def recover_ar4_worker(self, stage, brick):
        self.get_logger().info(f'[AR4 RECOVERY] Resuming from {stage} for brick {brick.id}')
        
        if self.emergency_stop or self.ar4_cancelled:
            self.ar4_busy = False
            return
        
        if stage == "PICK":
            # 1. MTC got us to the brick. Finalize the pick by closing the gripper manually.
            self.get_logger().info("Closing AR4 gripper to finalize MTC Pick...")
            await self.set_ar4_gripper(False) 
            
            if self.emergency_stop or self.ar4_cancelled:
                self.ar4_busy = False
                return
        
            # 2. Hand control back to the Task Server to finish the job
            self.get_logger().info("Executing AR4 PLACE...")
            plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=brick.place_pose)
            plc_goal.target_pose.position.z = 0.22 
            await self.send_action_goal(self.ar4_client, plc_goal)

            if self.emergency_stop or self.ar4_cancelled:
                self.ar4_busy = False
                return
            
        elif stage == "PLACE":
            # 1. MTC got us to the grid. Finalize the place by opening the gripper manually.
            self.get_logger().info("Opening AR4 gripper to finalize MTC Place...")
            await self.set_ar4_gripper(True) 

            if self.emergency_stop or self.ar4_cancelled:
                self.ar4_busy = False
                return
            
            # 2. Arm is empty. Tell the Task Server to go home.
            self.get_logger().info("Returning AR4 to HOME...")
            await self.send_action_goal(self.ar4_client, ExecuteTask.Goal(task_type="HOME"))

            if self.emergency_stop or self.ar4_cancelled:
                self.ar4_busy = False
                return
            
        self.ar4_stage = "DONE"
        self.ar4_busy = False
        self.ar4_recovering = False
        self.get_logger().info('[AR4 RECOVERY] Complete.')

    async def recover_abb_worker(self, stage, brick):
        self.get_logger().info(f'[ABB RECOVERY] Resuming from {stage} for brick {brick.id}')
        
        if self.emergency_stop or self.abb_cancelled:
            self.abb_busy = False
            return
        
        if stage == "PICK":
            # 1. Finalize the pick manually.
            self.get_logger().info("Closing ABB gripper to finalize MTC Pick...")
            await self.set_abb_gripper(False) 
            
            if self.emergency_stop or self.abb_cancelled:
                self.abb_busy = False
                return
        
            # 2. Execute PLACE
            self.get_logger().info("Executing ABB PLACE...")
            plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=brick.place_pose)
            plc_goal.target_pose.position.z = 0.24 
            await self.send_action_goal(self.abb_client, plc_goal)

            if self.emergency_stop or self.abb_cancelled:
                self.abb_busy = False
                return
            
        elif stage == "PLACE":
            # 1. Finalize the place manually.
            self.get_logger().info("Opening ABB gripper to finalize MTC Place...")
            await self.set_abb_gripper(True) 
            
            if self.emergency_stop or self.abb_cancelled:
                self.abb_busy = False
                return
            
            # 2. Go Home
            self.get_logger().info("Returning ABB to HOME...")
            await self.send_action_goal(self.abb_client, ExecuteTask.Goal(task_type="HOME"))

            if self.emergency_stop or self.abb_cancelled:
                self.abb_busy = False
                return
            
        self.abb_stage = "DONE"
        self.abb_busy = False
        self.abb_recovering = False
        self.get_logger().info('[ABB RECOVERY] Complete.')

def main(args=None):
    rclpy.init(args=args)
    node = HybridAssemblySupervisor()
    executor = MultiThreadedExecutor()
    
    # --- FIX THE EXECUTOR REFERENCE BUG (Update 7) ---
    node.executor = executor
    
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # --- FIX ROS SHUTDOWN CRASH (Update 8) ---
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()