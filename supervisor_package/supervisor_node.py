#!/usr/bin/env python3
"""
Unified Assembly Supervisor Node (Sim & Hardware)

This node serves as the high-level state machine orchestrating the assembly 
process using a dual-arm robotic setup (ABB and AR4). It handles sequential, 
parallel, and collaborative (handover) tasks.

Coordinate Logic is strictly isolated:
- SIMULATION MODE: 1:1 Pass-through. Relies entirely on mocked waypoints. 
  No transformations or Z-height modifications are applied.
- HARDWARE MODE: Applies explicit mathematical translations and rotational 
  quaternion overrides to compensate for physical calibration offsets.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
import tf2_geometry_msgs

# Custom Interfaces
from supervisor_package.srv import GetAssemblyPlan
from dual_arms_msgs.msg import GraspPoint 
from dual_arms_msgs.srv import GetGrasp, DetectBricks, GetHandoverZone, ExecuteMTCHandover, ResolveCollision
from dual_arms_msgs.action import ExecuteTask
from std_srvs.srv import SetBool
from std_msgs.msg import String

import math
import copy 
from tf2_ros import TransformException, Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

class UnifiedAssemblySupervisor(Node):
    def __init__(self):
        super().__init__('unified_supervisor')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('use_sim', True)
        self.declare_parameter('enable_mtc_mode', True)
        self.declare_parameter('handover_trigger_distance', 0.3)
        
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.handover_trigger_distance = self.get_parameter('handover_trigger_distance').get_parameter_value().double_value

        # Service Clients
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.grasp_pipeline_client = self.create_client(GetGrasp, 'grasp/get_grasp_point', callback_group=self.cb_group)
        self.gripper_client = self.create_client(SetBool, 'ar4_controller/set_gripper', callback_group=self.cb_group)
        self.abb_gripper_client = self.create_client(SetBool, 'abb_controller/set_gripper', callback_group=self.cb_group) 
        
        self.mtc_handover_client = self.create_client(ExecuteMTCHandover, 'mtc_controller/execute_handover', callback_group=self.cb_group)
        self.mtc_resolve_client = self.create_client(ResolveCollision, 'mtc_controller/resolve_collision', callback_group=self.cb_group)

        # Action Clients
        self.ar4_client = ActionClient(self, ExecuteTask, 'ar4_control', callback_group=self.cb_group)
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        # State tracking
        self.mtc_active = False
        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []
        self.known_brick_ids = set()
        self.detected_bricks = []
        self.current_grasp_point = None
        self.handover_pose = None
        self.operation_type = None                
        self.abb_grasp_point_for_handover = None  

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.ar4_active_target = None
        self.abb_active_target = None

        # Parallel State Locks
        self.ar4_busy = False
        self.abb_busy = False
        self.ar4_parallel_done = False
        self.abb_parallel_done = False
        self.ar4_timer = None
        self.abb_timer = None

        # Diagnostic tracking
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
        self._is_looping = False

        self.emergency_stop = False
        self.zone_sub = self.create_subscription(
            String,
            '/zone_status',
            self.zone_status_callback,
            10,
            callback_group=self.cb_group
        )

        self.plan_check_timer = None
        self.check_for_new_plans = True
        self.last_plan_count = 0
        self.handover_meeting_height = 0.157
        self.ar4_z_offset = 0.043
        self.abb_z_offset = 0.136

        mode_str = "SIMULATION (1:1 Pass-through)" if self.use_sim else "HARDWARE (Physical Offsets)"
        self.get_logger().info(f'Unified Assembly Supervisor Initialized in {mode_str} mode')
        
        # self.timer = self.create_timer(1.0, self.state_machine_loop, callback_group=self.cb_group)
        self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)
        self.plan_check_timer = self.create_timer(5.0, self.poll_for_new_plans, callback_group=self.cb_group)

    # ========================================================================
    # ARM STATE MONITORING
    # ========================================================================

    def _extract_yaw_safe(self, orientation):
        """
        Extract yaw (Z-rotation) from quaternion, avoiding gimbal lock issues.
        Uses the rotation matrix approach which is numerically more stable.
        References: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """
        try:
            # Normalize quaternion
            x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
            norm_sq = x**2 + y**2 + z**2 + w**2
            if norm_sq < 1e-8:
                return 0.0
            
            norm = math.sqrt(norm_sq)
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
            
            # Use atan2 on the rotation matrix elements to extract yaw
            # This works for all pitch angles, including ±180°
            # Formula from Wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion-based_formula
            # 
            # For 'xyz' intrinsic rotations:
            # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            
            # However, scipy uses 'xyz' convention where the third angle is yaw
            # For better numerical stability, use scipy but with normalization
            q = [x, y, z, w]
            euler = R.from_quat(q).as_euler('zyx', degrees=False)  # Use ZYX to avoid gimbal lock at pitch=±180
            yaw_rad = euler[0]  # First angle in ZYX is the Z-rotation (yaw)
            yaw_deg = math.degrees(yaw_rad)
            
            # Normalize to [0, 360) - convert negative angles to positive equivalents
            while yaw_deg < 0:
                yaw_deg += 360
            while yaw_deg >= 360:
                yaw_deg -= 360
            
            return yaw_deg
            
        except Exception as e:
            self.get_logger().warn(f"Error extracting yaw: {e}")
            return 0.0

    def _rpy_to_quat_normalized(self, roll_deg, pitch_deg, yaw_deg):
        """
        Convert Roll-Pitch-Yaw to quaternion using scipy with proper normalization.
        Uses 'zyx' convention to avoid gimbal lock at pitch=180°.
        Returns geometry_msgs/Quaternion
        """
        try:
            # Convert degrees to radians
            roll = math.radians(roll_deg)
            pitch = math.radians(pitch_deg)
            yaw = math.radians(yaw_deg)
            
            # Use ZYX convention to create the quaternion
            # This maps: zyx angles -> first angle is yaw (Z), then pitch (Y), then roll (X)
            # But scipy's from_euler with 'zyx' expects [yaw, pitch, roll] as input
            # So we need to invert the order
            euler_angles = [yaw, pitch, roll]  # ZYX order
            quat_array = R.from_euler('zyx', euler_angles, degrees=False).as_quat()
            
            quat_msg = Quaternion()
            quat_msg.x = quat_array[0]
            quat_msg.y = quat_array[1]
            quat_msg.z = quat_array[2]
            quat_msg.w = quat_array[3]
            
            return quat_msg
            
        except Exception as e:
            self.get_logger().error(f"Error converting RPY to quaternion: {e}")
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    def _multiply_quaternions(self, q1: Quaternion, q2: Quaternion):
        """
        Multiply two quaternions: result = q1 * q2
        This avoids gimbal lock issues when composing orientations.
        For gripper-down orientation (pitch=180°), use this instead of RPY reconstruction.
        
        Args:
            q1: First quaternion (the transformation to apply)
            q2: Second quaternion (the base grasp orientation)
        
        Returns:
            Resulting quaternion from multiplication
        """
        try:
            # Extract components
            x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
            x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
            
            # Quaternion multiplication formula: q1 * q2
            # (w1 + x1i + y1j + z1k) * (w2 + x2i + y2j + z2k)
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
            
            result = Quaternion()
            result.x = x
            result.y = y
            result.z = z
            result.w = w
            
            return result
        except Exception as e:
            self.get_logger().error(f"Error multiplying quaternions: {e}")
            return q2  # Return second quaternion as fallback

    def _get_pitch_down_quaternion(self):
        """
        Create a quaternion representing a 180° rotation around the Y-axis.
        This represents the gripper pointing downward.
        Uses a direct quaternion construction to avoid gimbal lock.
        
        Formula: For 180° rotation around Y-axis:
        q = [sin(90°), 0, 0, cos(90°)] (but normalized)
        Since sin(90°) = 1 and cos(90°) = 0, we get [1, 0, 0, 0] but that's incorrect...
        Actually: q = [0, 1, 0, 0] represents 180° rotation around Y-axis
        
        Returns:
            Quaternion representing pitch-down orientation
        """
        # 180° rotation around Y-axis
        # q = sin(angle/2) * [0, 1, 0] + cos(angle/2) * [0]
        # For 180°: sin(90°) = 1, cos(90°) = 0
        q_pitch_down = Quaternion()
        q_pitch_down.x = 0.0
        q_pitch_down.y = 1.0  # Rotation around Y-axis
        q_pitch_down.z = 0.0
        q_pitch_down.w = 0.0
        return q_pitch_down

    # ========================================================================
    # ARM STATE MONITORING (CONTINUED)
    # ========================================================================
    
    def get_actual_ee_orientation(self, arm_name: str):
        """Get the actual EE orientation from TF frame lookup"""
        try:
            # Determine frame names based on arm
            if arm_name == "AR4":
                ee_frame = "ar4_ee_link"
                base_frame = "ar4_base_link"
            elif arm_name == "ABB":
                ee_frame = "tool0"
                base_frame = "base_link"
            else:
                return None
            
            # Look up the transform
            transform = self.tf_buffer.lookup_transform(
                base_frame, ee_frame, 
                rclpy.time.Time()
            )
            
            # Extract yaw using TF2 convention (matches AR4 driver)
            quat = transform.transform.rotation
            yaw_deg = self._extract_yaw_safe(quat)
            
            return yaw_deg
        except Exception as e:
            self.get_logger().warn(f"Could not get {arm_name} EE orientation from TF: {e}")
            return None

    # ========================================================================
    # DYNAMIC TRANSFORMATION UTILITIES
    # ========================================================================

    def transform_pose_to_abb(self, input_pose: Pose):
        if self.use_sim:
            transformed_pose = Pose()
            
            # 1. Translation: Hardcode physical offsets from your URDF (X: 0.67, Y: 0.01).
            # We swap input X and Y to manually account for the camera's 90-degree rotation.
            transformed_pose.position.x = input_pose.position.y + 0.67
            transformed_pose.position.y = input_pose.position.x + 0.01
            transformed_pose.position.z = input_pose.position.z 
            transformed_pose.orientation = input_pose.orientation
            
            return transformed_pose
            
        try:
            transformed_pose = Pose()
            transformed_pose.position.x = input_pose.position.y + 0.674
            transformed_pose.position.y = input_pose.position.x + 0.033
            transformed_pose.position.z = input_pose.position.z
            transformed_pose.orientation = input_pose.orientation
            return transformed_pose
        except Exception as ex:
            self.get_logger().error(f'Transformation Error (HW): {ex}')
            return input_pose 

    # --- STRICTLY HARDWARE-ONLY HACKS ---
    def apply_ar4_hardware_pick_transform(self, pose: Pose):
        current_x = -pose.orientation.z
        current_y = -pose.orientation.w
        pose.orientation.x = current_x 
        pose.orientation.y = current_y
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose.position.z = 0.11
        return pose

    def apply_ar4_hardware_place_transform(self, pose: Pose):
        pose.orientation.x = pose.orientation.z
        pose.orientation.y = pose.orientation.w
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose.position.z = 0.12
        return pose

    def apply_abb_hardware_pick_transform(self, pose: Pose):
        current_x = -pose.orientation.z
        current_y = -pose.orientation.w
        pose.orientation.x = current_x
        pose.orientation.y = current_y
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose.position.z = 0.21
        return pose

    def apply_abb_hardware_place_transform(self, pose: Pose):
        current_x = -pose.orientation.z
        current_y = -pose.orientation.w
        pose.orientation.x = current_x
        pose.orientation.y = current_y
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose.position.z = 0.24
        return pose

    # ========================================================================
    # DELTA LOGIC (Rigid Body Match for Hardware)
    # ========================================================================

    def transform_position(self, place_pose, brick_pose, grasp_pose):
        dx = grasp_pose.position.x - brick_pose.position.x
        dy = grasp_pose.position.y - brick_pose.position.y
        
        def get_yaw(orient):
            # Check for zero-norm quaternion
            norm_sq = orient.x**2 + orient.y**2 + orient.z**2 + orient.w**2
            if norm_sq < 1e-6:  # Essentially zero
                return 0.0
            # Use the full quaternion as-is
            q = [orient.x, orient.y, orient.z, orient.w]
            return R.from_quat(q).as_euler('xyz')[2]

        yaw_start = get_yaw(brick_pose.orientation)
        yaw_end = get_yaw(place_pose.orientation)
        delta_yaw = yaw_end - yaw_start

        rotated_dx = dx * math.cos(delta_yaw) - dy * math.sin(delta_yaw)
        rotated_dy = dx * math.sin(delta_yaw) + dy * math.cos(delta_yaw)

        target_x = place_pose.position.x + rotated_dx
        target_y = place_pose.position.y + rotated_dy
        target_z = place_pose.position.z 

        return Pose().position.__class__(x=target_x, y=target_y, z=target_z)
    
    def transform_quaternion(self, place_pose, brick_pose, grasp_pose):
        if any(p is None for p in [place_pose, brick_pose, grasp_pose]):
            return None
        try:
            # 1. Safely extract all three yaw angles
            r_place = self._extract_yaw_safe(place_pose.orientation)
            r_brick = self._extract_yaw_safe(brick_pose.orientation)
            r_grasp = self._extract_yaw_safe(grasp_pose.orientation)
            
            # 2. Calculate the required final yaw 
            target_yaw_deg = r_grasp + (r_place - r_brick)
            
            # 3. Snap to the nearest 90 degree increment
            target_yaw_deg = round(target_yaw_deg / 90.0) * 90.0

            # 4. Normalize the angle between -180 and 180
            if target_yaw_deg > 180: target_yaw_deg -= 360
            if target_yaw_deg <= -180: target_yaw_deg += 360
            
            # 5. Build the final quaternion using the EXACT SAME method as the pick pose
            target_quat = self._rpy_to_quat_normalized(0.0, 180.0, target_yaw_deg)
            
            return target_quat
            
        except ValueError as e:
            self.get_logger().error(f"Quaternion Math Error: {e}")
            return None
    # ========================================================================
    # CORE LOGIC UTILITIES
    # ========================================================================

    async def ros_sleep(self, duration):
        future = rclpy.task.Future()
        timer = self.create_timer(duration, lambda: future.set_result(None) if not future.done() else None, callback_group=self.cb_group)
        await future
        self.destroy_timer(timer)

    def is_handover_operation(self, brick):
        if brick.start_side == brick.target_side: return False
        if brick.start_side not in ["AR4", "ABB"] or brick.target_side not in ["AR4", "ABB"]: return False
        return True

    def stop_parallel_workers(self):
        if self.ar4_timer:
            self.ar4_timer.cancel()
            self.ar4_timer = None
        if self.abb_timer:
            self.abb_timer.cancel()
            self.abb_timer = None
        self.ar4_busy = False
        self.abb_busy = False
        self.ar4_parallel_done = False
        self.abb_parallel_done = False

    def zone_status_callback(self, msg):
        # Ignore proximity alerts during intentional handovers
        if self.operation_type == "HANDOVER":
            return
        if "COLLISION_WARNING" in msg.data and not self.emergency_stop and not self.mtc_active and not self.ar4_recovering and not self.abb_recovering:
            self.get_logger().warn(f"⚠️ PROXIMITY ALERT! Interrupting AR4 in [{self.ar4_stage}] and ABB in [{self.abb_stage}]")
            self.mtc_active = True
            self.emergency_stop = True
            self.ar4_cancelled = True
            self.abb_cancelled = True
            
            if self.ar4_goal_handle: self.ar4_goal_handle.cancel_goal_async()
            if self.abb_goal_handle: self.abb_goal_handle.cancel_goal_async()

            self.state = "TRIGGER_MTC_SAFE_RESOLUTION"

    def poll_for_new_plans(self):
        self.executor.create_task(self._async_poll_for_new_plans())

    async def _async_poll_for_new_plans(self):
        try:
            if not self.gui_client.wait_for_service(timeout_sec=0.5): return
            req = GetAssemblyPlan.Request()
            result = await self.gui_client.call_async(req)
            
            if result is not None and len(result.plan) > 0:
                # --- REMOVED THE FLAWED LENGTH CHECK ---
                
                # Check permanent memory for new, unseen bricks
                new_bricks = [b for b in result.plan if b.id not in self.known_brick_ids]
                
                if new_bricks:
                    for b in new_bricks:
                        self.known_brick_ids.add(b.id) # Mark as seen!
                        
                    self.assembly_queue.extend(new_bricks)
                    if self.state == "WAIT_FOR_NEW_PLAN" or self.state == "DONE":
                        self.state = "DETECT"
        except Exception:
            pass

    async def send_action_goal(self, client, goal_msg):
        """
        Send an action goal and wait for result with timeout protection.
        Returns the result if successful, None otherwise.
        """
        try:
            # Check if server is available
            if not client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f'Action server not available for client')
                return None

            # Send goal
            self.get_logger().debug(f'Sending action goal to server...')
            goal_future = client.send_goal_async(goal_msg)
            goal_handle = await goal_future

            if not goal_handle.accepted:
                self.get_logger().error(f'Goal rejected by action server')
                return None

            self.get_logger().debug(f'Goal accepted by server, waiting for result...')

            # Track goal handle
            if client == self.ar4_client: 
                self.ar4_goal_handle = goal_handle
                if hasattr(goal_msg, 'target_pose'): 
                    self.ar4_active_target = goal_msg.target_pose
            elif client == self.abb_client: 
                self.abb_goal_handle = goal_handle
                if hasattr(goal_msg, 'target_pose'): 
                    self.abb_active_target = goal_msg.target_pose

            # Wait for result (ROS handles timeout through action protocol)
            result_future = goal_handle.get_result_async()
            result = await result_future

            # Clean up goal handle
            if goal_handle is not None:
                if client == self.ar4_client and self.ar4_goal_handle is not None and self.ar4_goal_handle == goal_handle: 
                    self.ar4_goal_handle = None
                elif client == self.abb_client and self.abb_goal_handle is not None and self.abb_goal_handle == goal_handle: 
                    self.abb_goal_handle = None

            # Check result status
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().debug(f'Goal succeeded')
                return result.result
            else:
                self.get_logger().error(f'Goal failed with status: {result.status}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error sending action goal: {e}')
            return None

    async def set_ar4_gripper(self, open_gripper: bool):
        if not self.gripper_client.wait_for_service(timeout_sec=2.0): return False
        req = SetBool.Request()
        req.data = open_gripper
        result = await self.gripper_client.call_async(req)
        return result.success
    
    async def set_abb_gripper(self, open_gripper: bool):
        if not self.abb_gripper_client.wait_for_service(timeout_sec=2.0): return False
        req = SetBool.Request()
        req.data = open_gripper
        result = await self.abb_gripper_client.call_async(req)
        return result.success
    
    # ========================================================================
    # PARALLEL WORKER EXECUTIONS (With Hardware Delta Logic)
    # ========================================================================

    async def execute_ar4_full_sequence(self, brick):
        self.get_logger().info(f'[PARALLEL] AR4 starting sequence for brick {brick.id}')
        
        # Log brick's actual orientation at the beginning
        brick_start_yaw = self._extract_yaw_safe(brick.pickup_pose.orientation)
        self.get_logger().info(f'[AR4] START - Brick actual orientation: {brick_start_yaw:.2f}°')
        
        req = GetGrasp.Request()
        req.brick_index = str(brick.id)
        res = await self.grasp_pipeline_client.call_async(req)
        if not res.success or self.emergency_stop or self.ar4_cancelled:
            self.get_logger().error(f'AR4 Grasp Pipeline failed: success={res.success}, emergency_stop={self.emergency_stop}, cancelled={self.ar4_cancelled}')
            return False
        # 1. Extract the raw 2D yaw from the camera's original grasp point
        raw_yaw_deg = self._extract_yaw_safe(res.grasp_point.pose.orientation) # use brick.pose.orientation in DETECT

        # 2. Add 90 degrees to account for the swapped X/Y axes in transform_pose_to_abb
        corrected_yaw_deg = raw_yaw_deg + 90.0
        # 3. Explicitly construct the hardware-safe quaternion (Roll=0, Pitch=180 points Z down, Yaw=Corrected)
        res.grasp_point.pose.orientation = self._rpy_to_quat_normalized(0.0, 180.0, corrected_yaw_deg)
        # Log initial grasp pose angle from grasping node
        initial_yaw = self._extract_yaw_safe(res.grasp_point.pose.orientation)
        self.get_logger().info(f'[AR4] 1. Initial grasp angle from node: {initial_yaw:.2f}°')

        grasp_pose = self.transform_pose_to_abb(res.grasp_point.pose)
        
        # Log after transform to ABB frame
        post_transform_yaw = self._extract_yaw_safe(grasp_pose.orientation)
        self.get_logger().info(f'[AR4] 2. After transform to ABB frame: {post_transform_yaw:.2f}°')
        
        place_pose = copy.deepcopy(brick.place_pose)

        if not self.use_sim:
            # Apply delta logic first!
            place_pose.position = self.transform_position(place_pose, brick.pickup_pose, grasp_pose)
            orientation = self.transform_quaternion(place_pose, brick.pickup_pose, grasp_pose)
            if orientation:
                place_pose.orientation.x = orientation.z
                place_pose.orientation.y = orientation.w
                place_pose.orientation.z = 0.0
                place_pose.orientation.w = 0.0
                
            # Then apply hardware physical offsets
            grasp_pose = self.apply_ar4_hardware_pick_transform(grasp_pose)
            place_pose = self.apply_ar4_hardware_place_transform(place_pose)
        else:
            place_pose.position = self.transform_position(place_pose, brick.pickup_pose, grasp_pose)
            target_place_orientation = self.transform_quaternion(place_pose, brick.pickup_pose, grasp_pose)
            grasp_pose.position.z = 0.11 
            place_pose.position.z = 0.12

            # Use the grasp orientation directly from the grasping module (no overrides)
            grasp_orientation_yaw = self._extract_yaw_safe(grasp_pose.orientation)
            self.get_logger().info(f'[AR4] 3. Final grasp orientation from module (no override): {grasp_orientation_yaw:.2f}°')

            # Use computed orientation for consistency with delta logic
            if target_place_orientation:
                place_pose.orientation = target_place_orientation
            else:
                place_pose.orientation = copy.deepcopy(grasp_pose.orientation)

        self.ar4_stage = "PICK"
        # Extract and log final pick angle
        pick_yaw = self._extract_yaw_safe(grasp_pose.orientation)
        self.get_logger().info(f'[AR4 PICK] 5. FINAL angle sent to arm: {pick_yaw:.2f}° | Brick ID: {brick.id}')
        pick_goal = ExecuteTask.Goal(task_type="PICK", target_pose=grasp_pose)
        result = await self.send_action_goal(self.ar4_client, pick_goal)
        if self.ar4_cancelled or result is None or self.emergency_stop:
            self.get_logger().error(f'AR4 PICK action failed: result={result}, cancelled={self.ar4_cancelled}, stopped={self.emergency_stop}')
            return False
        
        # Log actual EE orientation after pick completes
        actual_pick_yaw = self.get_actual_ee_orientation("AR4")
        if actual_pick_yaw is not None:
            pick_yaw_cmd = self._extract_yaw_safe(grasp_pose.orientation)
            self.get_logger().info(f'[AR4 PICK] COMPARISON - Commanded: {pick_yaw_cmd:.2f}° | Actual: {actual_pick_yaw:.2f}° | Diff: {abs(pick_yaw_cmd - actual_pick_yaw):.2f}°')
        
        self.ar4_stage = "PLACE"
        # Extract and log final place angle
        place_yaw = self._extract_yaw_safe(place_pose.orientation)
        self.get_logger().info(f'[AR4 PLACE] 5. FINAL angle sent to arm: {place_yaw:.2f}° | Brick ID: {brick.id}')
        plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=place_pose)
        result = await self.send_action_goal(self.ar4_client, plc_goal)
        if self.ar4_cancelled or result is None:
            self.get_logger().error(f'AR4 PLACE action failed: result={result}, cancelled={self.ar4_cancelled}')
            return False
        
        # Log actual EE orientation after place completes
        actual_place_yaw = self.get_actual_ee_orientation("AR4")
        if actual_place_yaw is not None:
            place_yaw_cmd = self._extract_yaw_safe(place_pose.orientation)
            self.get_logger().info(f'[AR4 PLACE] COMPARISON - Commanded: {place_yaw_cmd:.2f}° | Actual: {actual_place_yaw:.2f}° | Diff: {abs(place_yaw_cmd - actual_place_yaw):.2f}°')

        self.ar4_stage = "DONE"
        # Log final orientation when sequence completes
        final_yaw = self._extract_yaw_safe(place_pose.orientation)
        self.get_logger().info(f'[AR4] END - Final orientation after place: {final_yaw:.2f}°')
        return True
    
    async def execute_abb_full_sequence(self, brick):
        self.get_logger().info(f'[PARALLEL] ABB starting sequence for brick {brick.id}')
        if self.emergency_stop or self.abb_cancelled:
            self.get_logger().error(f'ABB sequence cancelled/stopped before start')
            return False
        
        # Log brick's actual orientation at the beginning
        brick_start_yaw = self._extract_yaw_safe(brick.pickup_pose.orientation)
        self.get_logger().info(f'[ABB] START - Brick actual orientation: {brick_start_yaw:.2f}°')

        req = GetGrasp.Request()
        req.brick_index = str(brick.id)
        res = await self.grasp_pipeline_client.call_async(req)
        
        # If the model fails or returns no grasp, stop the sequence safely
        if not res.success or self.emergency_stop or self.abb_cancelled: 
            self.get_logger().error(f"ABB Grasp Pipeline failed: success={res.success}, brick {brick.id}, stopped={self.emergency_stop}, cancelled={self.abb_cancelled}")
            return False

                # 1. Extract the raw 2D yaw from the camera's original grasp point
        raw_yaw_deg = self._extract_yaw_safe(res.grasp_point.pose.orientation) # use brick.pose.orientation in DETECT

        # 2. Add 90 degrees to account for the swapped X/Y axes in transform_pose_to_abb
        corrected_yaw_deg = raw_yaw_deg 

        # 3. Explicitly construct the hardware-safe quaternion (Roll=0, Pitch=180 points Z down, Yaw=Corrected)
        res.grasp_point.pose.orientation = self._rpy_to_quat_normalized(0.0, 180.0, corrected_yaw_deg)

        # Log initial grasp pose angle from grasping node
        initial_yaw = self._extract_yaw_safe(res.grasp_point.pose.orientation)
        self.get_logger().info(f'[ABB] 1. Initial grasp angle from node: {initial_yaw:.2f}°')

        # Successfully got the point from advanced_grasping_node!
        grasp_pose = self.transform_pose_to_abb(res.grasp_point.pose)
        
        # Log after transform to ABB frame
        post_transform_yaw = self._extract_yaw_safe(grasp_pose.orientation)
        self.get_logger().info(f'[ABB] 2. After transform to ABB frame: {post_transform_yaw:.2f}°')
        
        place_pose = copy.deepcopy(brick.place_pose)

        if not self.use_sim:
            # Apply delta logic
            place_pose.position = self.transform_position(place_pose, brick.pickup_pose, grasp_pose)
            orientation = self.transform_quaternion(place_pose, brick.pickup_pose, grasp_pose)
            if orientation:
                place_pose.orientation.x = orientation.z
                place_pose.orientation.y = orientation.w
                place_pose.orientation.z = 0.0
                place_pose.orientation.w = 0.0
                
            grasp_pose = self.apply_abb_hardware_pick_transform(grasp_pose)
            place_pose = self.apply_abb_hardware_place_transform(place_pose)
        else:
            place_pose.position = self.transform_position(place_pose, brick.pickup_pose, grasp_pose)
            target_place_orientation = self.transform_quaternion(place_pose, brick.pickup_pose, grasp_pose)
            grasp_pose.position.z = 0.21
            place_pose.position.z = 0.22

            # Use the grasp orientation directly from the grasping module (no overrides)
            grasp_orientation_yaw = self._extract_yaw_safe(grasp_pose.orientation)
            self.get_logger().info(f'[ABB] 3. Final grasp orientation from module (no override): {grasp_orientation_yaw:.2f}°')

            # Use computed orientation for consistency with delta logic
            if target_place_orientation:
                place_pose.orientation = target_place_orientation
            else:
                place_pose.orientation = copy.deepcopy(grasp_pose.orientation)

        self.abb_stage = "PICK"
        # Extract and log final pick angle
        pick_yaw = self._extract_yaw_safe(grasp_pose.orientation)
        self.get_logger().info(f'[ABB PICK] 5. FINAL angle sent to arm: {pick_yaw:.2f}° | Brick ID: {brick.id}')
        pick_goal = ExecuteTask.Goal(task_type="PICK", target_pose=grasp_pose)
        result = await self.send_action_goal(self.abb_client, pick_goal)
        if self.abb_cancelled or result is None or self.emergency_stop:
            self.get_logger().error(f'ABB PICK action failed: result={result}, cancelled={self.abb_cancelled}, stopped={self.emergency_stop}')
            return False
        
        # Log actual EE orientation after pick completes
        actual_pick_yaw = self.get_actual_ee_orientation("ABB")
        if actual_pick_yaw is not None:
            pick_yaw_cmd = self._extract_yaw_safe(grasp_pose.orientation)
            self.get_logger().info(f'[ABB PICK] COMPARISON - Commanded: {pick_yaw_cmd:.2f}° | Actual: {actual_pick_yaw:.2f}° | Diff: {abs(pick_yaw_cmd - actual_pick_yaw):.2f}°')
        
        self.abb_stage = "PLACE"
        # Extract and log final place angle
        place_yaw = self._extract_yaw_safe(place_pose.orientation)
        self.get_logger().info(f'[ABB PLACE] 5. FINAL angle sent to arm: {place_yaw:.2f}° | Brick ID: {brick.id}')
        plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=place_pose)
        result = await self.send_action_goal(self.abb_client, plc_goal)
        if self.abb_cancelled or result is None:
            self.get_logger().error(f'ABB PLACE action failed: result={result}, cancelled={self.abb_cancelled}')
            return False
        
        # Log actual EE orientation after place completes
        actual_place_yaw = self.get_actual_ee_orientation("ABB")
        if actual_place_yaw is not None:
            place_yaw_cmd = self._extract_yaw_safe(place_pose.orientation)
            self.get_logger().info(f'[ABB PLACE] COMPARISON - Commanded: {place_yaw_cmd:.2f}° | Actual: {actual_place_yaw:.2f}° | Diff: {abs(place_yaw_cmd - actual_place_yaw):.2f}°')
        
        self.abb_stage = "DONE"
        # Log final orientation when sequence completes
        final_yaw = self._extract_yaw_safe(place_pose.orientation)
        self.get_logger().info(f'[ABB] END - Final orientation after place: {final_yaw:.2f}°')
        return True

    async def execute_ar4_worker(self, brick):
        success = await self.execute_ar4_full_sequence(brick)
        
        # If it failed naturally (not because of an MTC collision interrupt)
        if not success and not self.ar4_cancelled and not self.emergency_stop:
            self.get_logger().error(f"❌ AR4 CRITICAL FAILURE on Brick {brick.id}! Halting System.")
            self.state = "EMERGENCY_STOP"
            
        self.ar4_busy = False 

    async def execute_abb_worker(self, brick):
        success = await self.execute_abb_full_sequence(brick)
        
        # If it failed naturally (not because of an MTC collision interrupt)
        if not success and not self.abb_cancelled and not self.emergency_stop:
            self.get_logger().error(f"❌ ABB CRITICAL FAILURE on Brick {brick.id}! Halting System.")
            self.state = "EMERGENCY_STOP"
            
        self.abb_busy = False

    # ========================================================================
    # RECOVERY WORKERS (WITH EXPLICIT HOME COMMAND)
    # ========================================================================

    async def recover_ar4_worker(self, stage, brick):
        self.get_logger().info(f'[AR4 RECOVERY] Resuming from {stage} for brick {brick.id}')
        if self.emergency_stop or self.ar4_cancelled:
            self.ar4_busy = False
            return
        
        if stage == "PICK":
            self.get_logger().info("Closing AR4 gripper to finalize MTC Pick...")
            await self.set_ar4_gripper(False) 
            
            if self.emergency_stop or self.ar4_cancelled:
                self.ar4_busy = False
                return
        
            self.get_logger().info("Executing AR4 PLACE...")
            place_pose = copy.deepcopy(brick.place_pose)
            if not self.use_sim:
                grasp_pose = self.transform_pose_to_abb(self.current_grasp_point.pose) if self.current_grasp_point else brick.pickup_pose
                place_pose.position = self.transform_position(place_pose, brick.pickup_pose, grasp_pose)
                orientation = self.transform_quaternion(place_pose, brick.pickup_pose, grasp_pose)
                if orientation:
                    place_pose.orientation.x = orientation.z
                    place_pose.orientation.y = orientation.w
                    place_pose.orientation.z = 0.0
                    place_pose.orientation.w = 0.0
                place_pose = self.apply_ar4_hardware_place_transform(place_pose)
            else:
                place_pose.position.z = 0.12
                # Compute brick-aware orientation for place
                target_place_orientation = self.transform_quaternion(place_pose, brick.pickup_pose, self.transform_pose_to_abb(self.current_grasp_point.pose) if self.current_grasp_point else brick.pickup_pose)
                if target_place_orientation:
                    place_pose.orientation = target_place_orientation
                else:
                    place_pose.orientation.x = 0.0
                    place_pose.orientation.y = 1.0
                    place_pose.orientation.z = 0.0
                    place_pose.orientation.w = 0.0

            plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=place_pose)
            await self.send_action_goal(self.ar4_client, plc_goal)

        elif stage == "PLACE":
            self.get_logger().info("Opening AR4 gripper to finalize MTC Place...")
            await self.set_ar4_gripper(True) 
            
            if self.emergency_stop or self.ar4_cancelled:
                self.ar4_busy = False
                return
            
            self.get_logger().info("Returning AR4 to HOME...")
            await self.send_action_goal(self.ar4_client, ExecuteTask.Goal(task_type="HOME"))
            
        elif stage in ["MOVE_TO_HANDOVER", "HOLDING_AT_HANDOVER"]:
            self.get_logger().info("AR4 interrupted during Handover. Retracting safely to HOME...")
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
            self.get_logger().info("Closing ABB gripper to finalize MTC Pick...")
            await self.set_abb_gripper(False) 
            
            if self.emergency_stop or self.abb_cancelled:
                self.abb_busy = False
                return
        
            self.get_logger().info("Executing ABB PLACE...")
            place_pose = copy.deepcopy(brick.place_pose)
            if not self.use_sim:
                grasp_pose = self.transform_pose_to_abb(self.current_grasp_point.pose) if self.current_grasp_point else brick.pickup_pose
                place_pose.position = self.transform_position(place_pose, brick.pickup_pose, grasp_pose)
                orientation = self.transform_quaternion(place_pose, brick.pickup_pose, grasp_pose)
                if orientation:
                    place_pose.orientation.x = orientation.z
                    place_pose.orientation.y = orientation.w
                    place_pose.orientation.z = 0.0
                    place_pose.orientation.w = 0.0
                place_pose = self.apply_abb_hardware_place_transform(place_pose)
            else:
                place_pose.position.z = 0.22
                # Compute brick-aware orientation for place
                target_place_orientation = self.transform_quaternion(place_pose, brick.pickup_pose, self.transform_pose_to_abb(self.current_grasp_point.pose) if self.current_grasp_point else brick.pickup_pose)
                if target_place_orientation:
                    place_pose.orientation = target_place_orientation
                else:
                    place_pose.orientation.x = 0.0
                    place_pose.orientation.y = 1.0
                    place_pose.orientation.z = 0.0
                    place_pose.orientation.w = 0.0 

            plc_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=place_pose)
            await self.send_action_goal(self.abb_client, plc_goal)

        elif stage == "PLACE":
            self.get_logger().info("Opening ABB gripper to finalize MTC Place...")
            await self.set_abb_gripper(True) 
            
            if self.emergency_stop or self.abb_cancelled:
                self.abb_busy = False
                return
            
            self.get_logger().info("Returning ABB to HOME...")
            await self.send_action_goal(self.abb_client, ExecuteTask.Goal(task_type="HOME"))

        elif stage in ["MOVE_TO_HANDOVER", "HOLDING_AT_HANDOVER"]:
            self.get_logger().info("ABB interrupted during Handover. Retracting safely to HOME...")
            await self.send_action_goal(self.abb_client, ExecuteTask.Goal(task_type="HOME"))
            
            if self.emergency_stop or self.abb_cancelled:
                self.abb_busy = False
                return
            
        self.abb_stage = "DONE"
        self.abb_busy = False
        self.abb_recovering = False
        self.get_logger().info('[ABB RECOVERY] Complete.')

    # ========================================================================
    # MAIN STATE MACHINE
    # ========================================================================

    async def state_machine_loop(self):

        if getattr(self, '_is_looping', False):
            return
        
        self._is_looping = True

        # self.timer.cancel()
        try:
            if self.state == "INIT":
                if not self.gui_client.wait_for_service(timeout_sec=1.0):
                    await self.ros_sleep(2.0)
                    return

                req = GetAssemblyPlan.Request()
                result = await self.gui_client.call_async(req)
                if result is not None and len(result.plan) > 0:
                    self.assembly_queue = result.plan
                    self.known_brick_ids = {b.id for b in result.plan}
                    self.state = "DETECT"
                else:
                    await self.ros_sleep(2.0)
                    # self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

            elif self.state == "DETECT":
                if not self.camera_client.wait_for_service(timeout_sec=1.0):
                    await self.ros_sleep(2.0)
                    return

                req = DetectBricks.Request()
                result = await self.camera_client.call_async(req)
                for brick in result.bricks:
                    # 1. Extract raw yaw
                    raw_yaw_deg = self._extract_yaw_safe(brick.pose.orientation)
                    
                    # 2. Add 90 degrees for the camera frame rotation
                    corrected_yaw_deg = raw_yaw_deg + 90.0
                    
                    # 3. Explicitly construct the hardware-safe quaternion
                    brick.pose.orientation = self._rpy_to_quat_normalized(0.0, 180.0, corrected_yaw_deg)
                    
                    # 4. Apply the physical X/Y translations
                    brick.pose = self.transform_pose_to_abb(brick.pose)
                
                self.detected_bricks = result.bricks
                self.handover_pose = self.transform_pose_to_abb(result.handover_pose)
                self.state = "PROCESS_NEXT"

            elif self.state == "PROCESS_NEXT":
                if not self.assembly_queue and not self.ar4_busy and not self.abb_busy:
                    self.state = "WAIT_FOR_NEW_PLAN"
                    return

                if not self.ar4_busy:
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
                            def ar4_callback():
                                if self.ar4_timer: self.ar4_timer.cancel()
                                self.executor.create_task(self.execute_ar4_worker(ar4_brick))
                            self.ar4_timer = self.create_timer(0.01, ar4_callback, callback_group=self.cb_group)

                if not self.abb_busy:
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
                            def abb_callback():
                                if self.abb_timer: self.abb_timer.cancel()
                                self.executor.create_task(self.execute_abb_worker(abb_brick))
                            self.abb_timer = self.create_timer(0.01, abb_callback, callback_group=self.cb_group)

            elif self.state == "GRASP_PIPELINE":
                if not self.grasp_pipeline_client.wait_for_service(timeout_sec=2.0):
                    self.state = "PROCESS_NEXT"
                    return

                grasp_req = GetGrasp.Request()
                grasp_req.brick_index = str(self.current_brick.id)
                grasp_result = await self.grasp_pipeline_client.call_async(grasp_req)

                if grasp_result.success:
                    # 1. Extract raw 2D yaw from the grasping node
                    raw_yaw_deg = self._extract_yaw_safe(grasp_result.grasp_point.pose.orientation)
                    
                    # 2. Add 90 degrees
                    corrected_yaw_deg = raw_yaw_deg + 90.0
                    
                    # 3. Explicitly construct the hardware-safe quaternion
                    grasp_result.grasp_point.pose.orientation = self._rpy_to_quat_normalized(0.0, 180.0, corrected_yaw_deg)
                    
                    self.current_grasp_point = grasp_result.grasp_point
                    
                    self.current_grasp_point = grasp_result.grasp_point
                    if self.is_handover_operation(self.current_brick):
                        self.get_logger().info(f'🔄 HANDOVER detected: {self.current_brick.start_side} → {self.current_brick.target_side}')
                        self.operation_type = "HANDOVER"
                        
                        self.handover_pose = Pose()
                        self.handover_pose.position.x = 0.56
                        self.handover_pose.position.y = 0.1
                        self.handover_pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
                        
                        
                        if self.current_brick.start_side == "AR4":
                            self.handover_pose.position.z = self.handover_meeting_height + self.ar4_z_offset
                            self.state = "AR4_PICK_FOR_HANDOVER"
                        else:
                            self.handover_pose.position.z = self.handover_meeting_height + self.abb_z_offset
                            self.state = "ABB_PICK_FOR_HANDOVER"

                    elif len(self.assembly_queue) > 0:
                        ar4_bricks = [b for b in self.assembly_queue if b.start_side == "AR4"]
                        abb_bricks = [b for b in self.assembly_queue if b.start_side == "ABB"]
                        if ar4_bricks and abb_bricks:
                            self.state = "INITIALIZE_PARALLEL_EXECUTION"
                        else:
                            self.state = f"EXECUTE_{self.current_brick.start_side}_DIRECT" if self.current_brick.start_side == "AR4" else "EXECUTE_ABB_PICK"
                    else:
                        self.state = f"EXECUTE_{self.current_brick.start_side}_DIRECT" if self.current_brick.start_side == "AR4" else "EXECUTE_ABB_PICK"
                else:
                    self.state = "PROCESS_NEXT"

            # ------------------------------------------------------------------------
            # TRUE PARALLEL STATE
            # ------------------------------------------------------------------------
            elif self.state == "INITIALIZE_PARALLEL_EXECUTION":
                brick1 = self.current_brick
                partner_side = "ABB" if brick1.start_side == "AR4" else "AR4"
                brick2 = next((b for b in self.assembly_queue if b.start_side == partner_side), None)
                
                if brick1 and brick2:
                    self.assembly_queue.remove(brick2) 
                    ar4_brick = brick1 if brick1.start_side == "AR4" else brick2
                    abb_brick = brick1 if brick1.start_side == "ABB" else brick2
                    
                    self.ar4_parallel_done = False
                    self.abb_parallel_done = False
                    self.ar4_busy = True
                    self.abb_busy = True 

                    async def run_ar4():
                        self.ar4_timer.cancel()
                        await self.execute_ar4_full_sequence(ar4_brick)
                        self.ar4_parallel_done = True
                        self.ar4_busy = False 
                        
                    async def run_abb():
                        self.abb_timer.cancel()
                        await self.execute_abb_full_sequence(abb_brick)
                        self.abb_parallel_done = True
                        self.abb_busy = False 
                        
                    self.ar4_timer = self.create_timer(0.01, run_ar4, callback_group=self.cb_group)
                    self.abb_timer = self.create_timer(0.01, run_abb, callback_group=self.cb_group)
                    
                    self.state = "WAIT_FOR_PARALLEL"
                else:
                    self.state = "EXECUTE_AR4_DIRECT" if self.current_brick.start_side == "AR4" else "EXECUTE_ABB_PICK"

            elif self.state == "WAIT_FOR_PARALLEL":
                if self.ar4_parallel_done and self.abb_parallel_done:
                    self.state = "PROCESS_NEXT"

            # ------------------------------------------------------------------------
            # HYBRID HANDOVER STATES
            # ------------------------------------------------------------------------
            elif self.state in ["AR4_PICK_FOR_HANDOVER", "ABB_PICK_FOR_HANDOVER"]:
                is_ar4 = (self.state == "AR4_PICK_FOR_HANDOVER")
                client = self.ar4_client if is_ar4 else self.abb_client
                
                grasp_pose = self.current_brick.pickup_pose
                
                if is_ar4:
                    self.ar4_stage = "PICK"
                    if not self.use_sim: grasp_pose = self.apply_ar4_hardware_pick_transform(grasp_pose)
                else:
                    self.abb_stage = "PICK"
                    if not self.use_sim: grasp_pose = self.apply_abb_hardware_pick_transform(grasp_pose)
                
                pick_goal = ExecuteTask.Goal(task_type="PICK", target_pose=grasp_pose)
                result = await self.send_action_goal(client, pick_goal)
                if not result or not result.success:
                    self.state = "RECOVERY"
                    return
                
                if is_ar4: self.ar4_stage = "MOVE_TO_HANDOVER"
                else: self.abb_stage = "MOVE_TO_HANDOVER"
                    
                give_goal = ExecuteTask.Goal(task_type="INTERMEDIATE_GIVE", target_pose=self.handover_pose)
                result = await self.send_action_goal(client, give_goal)
                if not result or not result.success:
                    self.state = "RECOVERY"
                    return
                
                if is_ar4: self.ar4_stage = "HOLDING_AT_HANDOVER"
                else: self.abb_stage = "HOLDING_AT_HANDOVER"
                self.state = "QUERY_HANDOVER_GRASP"

            elif self.state == "QUERY_HANDOVER_GRASP":
                receiving_arm = self.current_brick.target_side
                
                grasp_req = GetGrasp.Request()
                grasp_req.brick_index = str(self.current_brick.id)
                grasp_result = await self.grasp_pipeline_client.call_async(grasp_req)
                
                if grasp_result.success:
                    grasp_for_receiver = grasp_result.grasp_point
                    transformed_receiver_grasp = self.transform_pose_to_abb(grasp_for_receiver.pose)
                    
                    # FORCE MID-AIR Z-HEIGHT MATCH
                    if self.current_brick.start_side == "AR4":
                        transformed_receiver_grasp.position.z = self.handover_meeting_height + self.abb_z_offset

                    if self.current_brick.start_side == "ABB":
                        transformed_receiver_grasp.position.z = self.handover_meeting_height + self.ar4_z_offset
                    
                    if receiving_arm == "AR4" and not self.use_sim:
                        transformed_receiver_grasp = self.apply_ar4_hardware_pick_transform(transformed_receiver_grasp)
                    elif receiving_arm == "ABB" and not self.use_sim:
                        transformed_receiver_grasp = self.apply_abb_hardware_pick_transform(transformed_receiver_grasp)

                    self.abb_grasp_point_for_handover = grasp_for_receiver
                    self.abb_grasp_point_for_handover.pose = transformed_receiver_grasp
                    self.state = "RETRIEVE_FROM_HANDOVER"
                else:
                    self.state = "RECOVERY"
                    return

            elif self.state == "RETRIEVE_FROM_HANDOVER":
                waiting_arm = self.current_brick.start_side
                receiving_arm = self.current_brick.target_side
                
                receiver_client = self.ar4_client if receiving_arm == "AR4" else self.abb_client
                giver_client = self.ar4_client if waiting_arm == "AR4" else self.abb_client
                
                if receiving_arm == "AR4": self.ar4_stage = "MOVE_TO_HANDOVER"
                else: self.abb_stage = "MOVE_TO_HANDOVER"
                
                take_goal = ExecuteTask.Goal(task_type="INTERMEDIATE_TAKE", target_pose=self.abb_grasp_point_for_handover.pose)
                result = await self.send_action_goal(receiver_client, take_goal)
                if not result or not result.success:
                    self.state = "RECOVERY"
                    return
                
                await self.send_action_goal(giver_client, ExecuteTask.Goal(task_type="RELEASE"))
                await self.send_action_goal(giver_client, ExecuteTask.Goal(task_type="HOME"))
                
                if waiting_arm == "AR4": self.ar4_stage = "DONE"
                else: self.abb_stage = "DONE"
                    
                if receiving_arm == "AR4": self.ar4_stage = "HOLDING_AT_HANDOVER"
                else: self.abb_stage = "HOLDING_AT_HANDOVER"
                self.state = "PLACE_FROM_HANDOVER"

            elif self.state == "PLACE_FROM_HANDOVER":
                receiving_arm = self.current_brick.target_side
                is_ar4_placing = (receiving_arm == "AR4")
                placer_client = self.ar4_client if is_ar4_placing else self.abb_client
                
                if is_ar4_placing: self.ar4_stage = "PLACE"
                else: self.abb_stage = "PLACE"
                
                place_pose = copy.deepcopy(self.current_brick.place_pose)
                
                if not self.use_sim: 
                    # Apply delta math for Handover placement
                    place_pose.position = self.transform_position(place_pose, self.current_brick.pickup_pose, self.abb_grasp_point_for_handover.pose)
                    orientation = self.transform_quaternion(place_pose, self.current_brick.pickup_pose, self.abb_grasp_point_for_handover.pose)
                    if orientation:
                        place_pose.orientation.x = orientation.z
                        place_pose.orientation.y = orientation.w
                        place_pose.orientation.z = 0.0
                        place_pose.orientation.w = 0.0
                        
                    if is_ar4_placing:
                        place_pose = self.apply_ar4_hardware_place_transform(place_pose)
                    else:
                        place_pose = self.apply_abb_hardware_place_transform(place_pose)
                else:
                    place_pose.position.z = 0.12 if is_ar4_placing else 0.22

                place_pose.orientation.x = 0.0
                place_pose.orientation.y = 1.0
                place_pose.orientation.z = 0.0
                place_pose.orientation.w = 0.0
                
                place_goal = ExecuteTask.Goal(task_type="PLACE", target_pose=place_pose)
                result = await self.send_action_goal(placer_client, place_goal)
                if not result or not result.success:
                    self.state = "RECOVERY"
                    return
                
                if is_ar4_placing: self.ar4_stage = "DONE"
                else: self.abb_stage = "DONE"
                
                # --- NEW: CLEAR OPERATION TYPE ---
                self.operation_type = "SEQUENTIAL"
                self.state = "PROCESS_NEXT"

            # ------------------------------------------------------------------------
            # EMERGENCY & MTC RESOLUTION STATES
            # ------------------------------------------------------------------------
            elif self.state == "TRIGGER_MTC_SAFE_RESOLUTION":
                self.get_logger().info('Engaging MTC to resolve proximity conflict safely...')
                
                if not self.mtc_resolve_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('MTC Resolution service missing! Halting safely.')
                    self.state = "EMERGENCY_STOP"
                    return
                
                req = ResolveCollision.Request()
                req.ar4_target_pose = self.ar4_active_target if self.ar4_active_target else Pose()
                req.abb_target_pose = self.abb_active_target if self.abb_active_target else Pose()
                
                result = await self.mtc_resolve_client.call_async(req)
                
                if result and result.success:
                    self.get_logger().info('✅ MTC Resolution Complete. Waiting for physical clearance...')
                    await self.ros_sleep(1.5)
                    self.get_logger().info(f"🔄 EXITING MTC. AR4 was in [{self.ar4_stage}], ABB was in [{self.abb_stage}]")

                    self.mtc_active = False
                    self.emergency_stop = False
                    self.ar4_active_target = None
                    self.abb_active_target = None
                    self.ar4_cancelled = False
                    self.abb_cancelled = False

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

                    self.state = "PROCESS_NEXT"
                else:
                    self.get_logger().error('MTC Resolution Failed.')
                    self.state = "EMERGENCY_STOP"

            elif self.state == "EMERGENCY_STOP":
                self.get_logger().error("SYSTEM HALTED - Manual Reset Required")
                await self.ros_sleep(3.0)

            elif self.state == "WAIT_FOR_NEW_PLAN":
                await self.ros_sleep(2.0) 
                if self.assembly_queue and not self.ar4_busy and not self.abb_busy:
                    self.state = "DETECT"
                    return

        except Exception as e:
            self.get_logger().error(f'State machine error: {e}')

        finally:
            self._is_looping = False
            # ALWAYS keep the heartbeat alive, regardless of state
            # self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedAssemblySupervisor()
    executor = MultiThreadedExecutor()
    node.executor = executor
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()