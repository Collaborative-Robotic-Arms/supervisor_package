#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_geometry_msgs # Required for do_transform_pose
from action_msgs.msg import GoalStatus

# Custom Interfaces
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.action import MoveToPose, AlignToTarget
from dual_arms_msgs.msg import GraspPoint 
from dual_arms_msgs.srv import GetGrasp , DetectBricks
from geometry_msgs.msg import TransformStamped, Pose # Added Pose to imports
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from dual_arms_msgs.action import ExecuteTask
from std_srvs.srv import SetBool

# --- NEW TF2 IMPORTS ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation as R

import math
# -----------------------

class AssemblySupervisor(Node):
    def __init__(self):
        super().__init__('supervisor')

        self.cb_group = ReentrantCallbackGroup()
        
        # --- GRIPPER INITIALIZATION ---
        self.declare_parameter('use_sim', True)
        use_sim_value = self.get_parameter('use_sim').get_parameter_value().bool_value
        # self.gripper = GripperManager(self, use_sim=use_sim_value)

        # --- TF2 INITIALIZATION ---
        # Broadcaster: Defines the static relationship between the robot base and camera lens
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # static_transform = TransformStamped()
        # static_transform.header.stamp = self.get_clock().now().to_msg()
        # static_transform.header.frame_id = 'base_link'      # Parent Frame
        # static_transform.child_frame_id = 'camera' # Child Frame

        # # Measured physical offsets in meters
        # static_transform.transform.translation.x = -0.05
        # static_transform.transform.translation.y = 0.67
        # static_transform.transform.translation.z = 0.769

        # # Identity rotation (aligned axes)
        # static_transform.transform.rotation.x = 0.0
        # static_transform.transform.rotation.y = 0.0
        # static_transform.transform.rotation.z = 0.0
        # static_transform.transform.rotation.w = 1.0

        # self.static_broadcaster.sendTransform(static_transform)

        # Listener: Setup the buffer to receive and calculate the actual transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('TF2 Static Broadcaster and Listener ready.')
        # ---------------------------

        # --- CLIENTS ---
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.ar4_point_client = ActionClient(self, MoveToPose, 'ar4_point_control', callback_group=self.cb_group)
        self.ar4_vs_client = ActionClient(self, AlignToTarget, 'ar4_visual_servo', callback_group=self.cb_group)
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)
        self.grasp_client = ActionClient(self, MoveToPose, 'grasp_pipeline', callback_group=self.cb_group)
        self.grasp_pipeline_client = self.create_client(GetGrasp, 'grasp/get_grasp_point', callback_group=self.cb_group) 
        self.gripper_client = self.create_client(SetBool, 'ar4_gripper/set', callback_group=self.cb_group)
        self.get_logger().info('Supervisor Initialized. Waiting for services...')

        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []
        self.detected_bricks = [] # Array of dual_arms_msgs/Brick
        self.current_grasp_point = None # Stores the grasp for the current task

        self.timer = self.create_timer(1.0, self.state_machine_loop, callback_group=self.cb_group)

    # --- NEW HELPER METHOD ---
    def transform_pose_to_abb(self, input_pose:Pose):
        """Standard TF2 transformation from Camera frame to ABB base_link."""
        try:
            # Lookup the transformation broadcasted in __init__
            # t = self.tf_buffer.lookup_transform(
            #     'base_link', 
            #     'camera', 
            #     rclpy.time.Time()) # Get the latest available transform

            transformed_pose = Pose()
            transformed_pose.position.x = input_pose.position.y + 0.674
            transformed_pose.position.y = input_pose.position.x + 0.033

            transformed_pose.orientation.x = input_pose.orientation.x
            transformed_pose.orientation.y = input_pose.orientation.y
            transformed_pose.orientation.z = input_pose.orientation.z
            transformed_pose.orientation.w = input_pose.orientation.w

            # Use tf2_geometry_msgs to translate/rotate the pose
            # transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, t)

            return transformed_pose

        except TransformException as ex:
            self.get_logger().error(f'TF2 Error: {ex}')
            return input_pose # Fallback to original pose
    # -------------------------

    def transform_quaternion(self, place_pose, brick_pose, grasp_pose):

        # 1. Check for NoneType (Prevents the 'NoneType' error)
        if any(p is None for p in [place_pose, brick_pose, grasp_pose]):
            self.get_logger().error("One of the input poses is None!")
            return None

        try:
            # Helper to ensure we don't have [0,0,0,0]
            def get_valid_quat(orient):
                if orient.z == 0.0 and orient.w == 0.0:
                    return [0, 0, 0, 1] # Return Identity (no rotation)
                return [0, 0, orient.z, orient.w]

            current_x = -grasp_pose.orientation.x
            current_y = -grasp_pose.orientation.y
            grasp_pose.orientation.z = current_x
            grasp_pose.orientation.w = current_y
            grasp_pose.orientation.x = 0.0
            grasp_pose.orientation.y = 0.0

            # place_pose.orientation.x = 0.0
            # place_pose.orientation.y = 0.0
            # place_pose.orientation.z = 0.0
            # place_pose.orientation.w = 1.0
            
            self.get_logger().info(f"Input Quaternions -> Place: z={place_pose.orientation.z:.3f}, w={place_pose.orientation.w:.3f} | Brick: z={brick_pose.orientation.z:.3f}, w={brick_pose.orientation.w:.3f} | Grasp: z={grasp_pose.orientation.z:.3f}, w={grasp_pose.orientation.w:.3f}")
            # 2. Convert to Euler using the helper
            r_place = R.from_quat(get_valid_quat(place_pose.orientation)).as_euler('xyz', degrees=True)[2]
            r_brick = R.from_quat(get_valid_quat(brick_pose.orientation)).as_euler('xyz', degrees=True)[2]
            
            r_grasp = R.from_quat(get_valid_quat(grasp_pose.orientation)).as_euler('xyz', degrees=True)[2]
            self.get_logger().info(f"Euler Angles (deg) -> Place: {r_place:.2f}, Brick: {r_brick:.2f}, Grasp: {r_grasp:.2f}")
            # 3. Thesis logic
            target_yaw_deg = r_grasp + (r_place - r_brick)

            target_yaw_deg = round(target_yaw_deg / 90.0) * 90.0

            # Normalize to range [-180, 180] to prevent the robot from spinning 270 degrees 
            # when it could just rotate -90 degrees.
            if target_yaw_deg > 180:
                target_yaw_deg -= 360
            if target_yaw_deg <= -180:
                target_yaw_deg += 360
            
            # 4. Convert back
            target_quat_array = R.from_euler('z', target_yaw_deg, degrees=True).as_quat()

            self.get_logger().info(f"Target Yaw (deg): {target_yaw_deg:.2f} => Quaternion(z={target_quat_array[2]:.3f}, w={target_quat_array[3]:.3f})")

            # 5. Map back to a ROS Quaternion object (Prevents attribute errors later)
            from geometry_msgs.msg import Quaternion
            target_quat = Quaternion()
            target_quat.x = 0.0
            target_quat.y = 0.0
            target_quat.z = target_quat_array[2]
            target_quat.w = target_quat_array[3]
            
            self.get_logger().info(f"Transformed Quaternion: x={target_quat.x:.3f}, y={target_quat.y:.3f}, z={target_quat.z:.3f}, w={target_quat.w:.3f}")
            return target_quat

        except ValueError as e:
            self.get_logger().error(f"Quaternion Math Error: {e}")
            return None
    
    def transform_position(self, place_pose, brick_pose, grasp_pose):
        """
        Determines the ending position of the grasp point based on the 
        rigid transformation (translation + rotation) of the brick.
        """
        # 1. Calculate the relative translation (Vector from Brick to Grasp)
        # in the Global Frame
        dx = grasp_pose.position.x - brick_pose.position.x
        dy = grasp_pose.position.y - brick_pose.position.y
        
        # 2. Calculate the rotation of the brick from start to end
        # We use the Z-axis rotation (Yaw) for 2D rigid body logic
        def get_yaw(orient):
            # Handles cases where quat might be [0,0,0,0]
            q = [orient.x, orient.y, orient.z, orient.w]
            if orient.w == 0.0 and orient.z == 0.0:
                return 0.0
            return R.from_quat(q).as_euler('xyz')[2]

        yaw_start = get_yaw(brick_pose.orientation)
        yaw_end = get_yaw(place_pose.orientation)
        delta_yaw = yaw_end - yaw_start

        # 3. Rotate the relative offset vector by the change in yaw
        # This accounts for the link 'rotating' in 2D space
        # x' = x cos(theta) - y sin(theta)
        # y' = x sin(theta) + y cos(theta)
        rotated_dx = dx * math.cos(delta_yaw) - dy * math.sin(delta_yaw)
        rotated_dy = dx * math.sin(delta_yaw) + dy * math.cos(delta_yaw)

        # 4. Apply the rotated offset to the final brick position
        target_x = place_pose.position.x + rotated_dx
        target_y = place_pose.position.y + rotated_dy
        target_z = 0.21 #place_pose.position.z + (grasp_pose.position.z - brick_pose.position.z)

        self.get_logger().info(f"Rigid Transform -> DeltaYaw: {math.degrees(delta_yaw):.2f}°")
        self.get_logger().info(f"Original Offset: dx={dx:.3f}, dy={dy:.3f} | Rotated Offset: dx={rotated_dx:.3f}, dy={rotated_dy:.3f}")
        self.get_logger().info(f"Place Pose: x={place_pose.position.x:.3f}, y={place_pose.position.y:.3f} | Brick Pose: x={brick_pose.position.x:.3f}, y={brick_pose.position.y:.3f} | Grasp Pose: x={grasp_pose.position.x:.3f}, y={grasp_pose.position.y:.3f}")
        self.get_logger().info(f"Target Pos: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")
        
        # Return the position object
        return Pose().position.__class__(x=target_x, y=target_y, z=target_z)
    

    def transform_position2(self, place_pose, brick_pose, grasp_pose):
      
        # Simple translation logic based on your thesis requirements
        delta_x = grasp_pose.position.x - brick_pose.position.x
        delta_y = grasp_pose.position.y - brick_pose.position.y
        delta_z = grasp_pose.position.z - brick_pose.position.z

        target_x = place_pose.position.x + delta_x
        target_y = place_pose.position.y + delta_y
        target_z = place_pose.position.z + delta_z

        self.get_logger().info(f"Transformed Position: Δx({delta_x:.3f}), Δy({delta_y:.3f}), Δz({delta_z:.3f}) => Target(x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f})")
        return Pose().position.__class__(x=target_x, y=target_y, z=target_z)
        
    async def state_machine_loop(self):
        self.timer.cancel()
        try:
            # =========================
            # STATE 1: INIT → DETECTION
            # =========================
            if self.state == "INIT":
                self.get_logger().info('Requesting Assembly Plan from GUI...')
                
                # Check service availability without blocking the executor indefinitely
                if not self.gui_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for GUI node...')
                    self.timer = self.create_timer(2.0, self.state_machine_loop, callback_group=self.cb_group)
                    return

                req = GetAssemblyPlan.Request()
                result = await self.gui_client.call_async(req)
                
                # --- FIX: Only proceed if the plan actually contains bricks ---
                if result is not None and len(result.plan) > 0:
                    self.assembly_queue = result.plan
                    self.get_logger().info(f'Plan received! {len(self.assembly_queue)} bricks to process.')
                    self.state = "DETECT"
                else:
                    self.get_logger().warn('Assembly plan is empty or service failed. Retrying in 2 seconds...')
                    # Keep state as "INIT" and retry after a delay
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
                
                # Transform each brick to the ABB base frame
                for brick in result.bricks:
                    brick.pose = self.transform_pose_to_abb(brick.pose)
                self.detected_bricks = result.bricks
                
                # Transform handover pose to the ABB base frame
                self.handover_pose = self.transform_pose_to_abb(result.handover_pose)

                self.state = "PROCESS_NEXT"

            # =========================
            # STATE 2: PROCESS NEXT & GRASP PIPELINE
            # =========================
            elif self.state == "PROCESS_NEXT":
                if not self.assembly_queue:
                    self.get_logger().info('All tasks complete!')
                    self.state = "DONE"
                    return

                self.current_brick = self.assembly_queue.pop(0)
                # Transition to the newly added Grasp Pipeline state
                self.state = "GRASP_PIPELINE"

            elif self.state == "GRASP_PIPELINE":
                self.get_logger().info(f'Calling GetGrasp Service for Brick ID: {self.current_brick.id}') 

                # 1. Wait for the service
                if not self.grasp_pipeline_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error('Grasp Pipeline service not available!')
                    self.state = "PROCESS_NEXT" # Skip or handle error
                    return

                # 2. Create the request using the brick ID
                grasp_req = GetGrasp.Request()
                grasp_req.brick_index = str(self.current_brick.id) # 
                
                # 3. Call the service and wait for the result
                grasp_result = await self.grasp_pipeline_client.call_async(grasp_req) # 

                if grasp_result.success:
                    # --- MODIFICATION: Transform Grasp point ---
                    # self.current_grasp_point = grasp_result.grasp_point
                    
                    # Capture and transform the specific grasp pose to ABB base link
                    raw_grasp = grasp_result.grasp_point
                    self.current_grasp_point = raw_grasp
                    raw_grasp.pose = self.transform_pose_to_abb(raw_grasp.pose)
                    self.current_grasp_point.pose.position.z = 0.22

    
                    # self.current_grasp_point.pose.orientation.x = raw_grasp.pose.orientation.w
                    # self.current_grasp_point.pose.orientation.y = raw_grasp.pose.orientation.z
                    # self.current_grasp_point.pose.orientation.z = 0.0
                    # self.current_grasp_point.pose.orientation.w = 0.0
                    # self.current_grasp_point.pose.orientation.x = raw_grasp.pose.orientation.x
                    # self.current_grasp_point.pose.orientation.y = raw_grasp.pose.orientation.y
                    # self.current_grasp_point.pose.orientation.z = raw_grasp.pose.orientation.z
                    # self.current_grasp_point.pose.orientation.w = raw_grasp.pose.orientation.w



                    #
                    # --------------------------------------------
                    self.get_logger().info(
                    f"RAW GRASP -> Pos: [x={self.current_grasp_point.pose.position.x:.3f}, y={self.current_grasp_point.pose.position.y:.3f}, z={self.current_grasp_point.pose.position.z:.3f}] "
                    f"| Orient: [x={self.current_grasp_point.pose.orientation.x:.3f}, y={self.current_grasp_point.pose.orientation.y:.3f}, z={self.current_grasp_point.pose.orientation.z:.3f}, w={self.current_grasp_point.pose.orientation.w:.3f}]")
                    
                    # --------------------------------------------

                    self.get_logger().info(f'Grasp retrieved and transformed. Quality: {self.current_grasp_point.quality}') # 
                    
                    # Branch to the correct arm based on current_brick start_side 
                    if self.current_brick.start_side == "ABB":
                        self.state = "EXECUTE_ABB_PICK"
                    elif self.current_brick.start_side == "HANDOVER": # Or whatever string logic you use for handover
                        self.state = "HANDOVER_SEQUENCE"
                    elif self.current_brick.start_side == "AR4":
                        self.state = "EXECUTE_AR4_DIRECT"
                    else:
                        self.get_logger().error(f"Unknown start_side: {self.current_brick.start_side}")
                else:
                    self.get_logger().error(f'Failed to get grasp for brick {self.current_brick.id}')
                    self.state = "PROCESS_NEXT"

            # =========================
            # STATE 3: AR4 PICK & PLACE
            # =========================
            elif self.state in ["EXECUTE_AR4_DIRECT", "AR4_PICK_FOR_HANDOVER"]:
                self.get_logger().info(f'Starting AR4 Pick Sequence for {self.current_brick.id}')

                # Step A: Approach
                goal_msg = MoveToPose.Goal()
                goal_msg.target_pose = self.current_grasp_point.pose 
                goal_msg.strategy = "APPROACH_OFFSET"
                action_result = await self.send_action_goal(self.ar4_point_client, goal_msg)
                if await self.check_and_recover(action_result, self.state): 
                    return

                await self.set_ar4_gripper(open_gripper=True)
                await self.set_ar4_gripper(open_gripper=False)
                
                # Step B: Visual Servoing
                self.get_logger().info('Switching to Visual Servoing...')
                vs_goal = AlignToTarget.Goal()
                vs_goal.object_id = self.current_brick.type
                action_result = await self.send_action_goal(self.ar4_vs_client, vs_goal)
                if await self.check_and_recover(action_result, self.state): 
                    return

                # Step C: Lower and Grasp
                self.get_logger().info('Lowering and Grasping...')
                grasp_goal = MoveToPose.Goal()
                grasp_goal.target_pose = self.current_grasp_point.pose 
                grasp_goal.strategy = "GRASP"
                action_result = await self.send_action_goal(self.ar4_point_client, grasp_goal)
                if await self.check_and_recover(action_result, self.state):
                    return
                # self.gripper.set_ar4_grip('CLOSE')
                self.state = "AR4_PLACE_ON_GRID" if self.state == "EXECUTE_AR4_DIRECT" else "HANDOVER_EXECUTION"

            elif self.state == "AR4_PLACE_ON_GRID":
                place_goal = MoveToPose.Goal()
                place_goal.target_pose = self.current_brick.place_pose 
                place_goal.strategy = "PLACE"
                action_result = await self.send_action_goal(self.ar4_point_client, place_goal)
                # self.gripper.set_ar4_grip('OPEN')
                if await self.check_and_recover(action_result, self.state):
                    return
                self.state = "PROCESS_NEXT"

            # =========================
            # STATE 4: ABB PICK & PLACE
            # =========================
            elif self.state == "EXECUTE_ABB_PICK":
                self.get_logger().info(f'Starting ABB Pick for {self.current_brick.id}')

                abb_pick_goal = ExecuteTask.Goal()
                abb_pick_goal.task_type = "PICK"
                
                                # Your current values
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
                    self.get_logger().info('Targeting GraspPoint pose.')
                    abb_pick_goal.target_pose = self.current_grasp_point.pose
                else:
                    abb_pick_goal.target_pose = self.current_brick.pickup_pose 
                
                action_result = await self.send_action_goal(self.abb_client, abb_pick_goal)
                # if await self.check_and_recover(action_result, self.state):
                #     return
                self.state = "EXECUTE_ABB_PLACE"
                
            elif self.state == "EXECUTE_ABB_PLACE":
                abb_place_goal = ExecuteTask.Goal()
                abb_place_goal.task_type = "PLACE"
                abb_place_goal.target_pose = self.current_brick.place_pose 


                # abb_place_goal.target_pose = self.current_grasp_point + (self.current_brick.place_pose - self.current_brick.pickup_pose)
                # abb_place_goal.target_pose = self.current_brick.place_pose 

                abb_place_goal.target_pose.position = self.transform_position(grasp_pose=self.current_grasp_point.pose,
                                                                        brick_pose=self.current_brick.pickup_pose,
                                                                        place_pose=self.current_brick.place_pose)

                orientation = self.transform_quaternion(grasp_pose=self.current_grasp_point.pose,
                                                                brick_pose=self.current_brick.pickup_pose,
                                                                place_pose=self.current_brick.place_pose)
                
                abb_place_goal.target_pose.orientation.x = orientation.z
                abb_place_goal.target_pose.orientation.y = orientation.w
                
                # abb_place_goal.target_pose.orientation.x = self.current_brick.place_pose.orientation.z
                # abb_place_goal.target_pose.orientation.y = self.current_brick.place_pose.orientation.w
                abb_place_goal.target_pose.orientation.z = 0.0
                abb_place_goal.target_pose.orientation.w = 0.0
                # abb_place_goal.target_pose = self.current_brick.place_pose
                self.get_logger().info(f'abb_place_goal.target_pose.position.z before setting: {abb_place_goal.target_pose.orientation.z:.3f}')
                abb_place_goal.target_pose.position.z = 0.22  # Must come from Assembly plan correct (0.21 if first layer, 0.26 if second layer)
                # abb_place_goal.target_pose.orientation.y = 1.0 # Current_grasp_point.orientation + (self.current_brick.place_pose.orientation - self.current_brick.pickup_pose.orientation)
                # abb_place_goal.target_pose.orientation.x = 0.0
                # abb_place_goal.target_pose.orientation.z = 0.0
                # abb_place_goal.target_pose.orientation.w = 0.0
                
                
                # # --- NEW LOGGER ---
                p = abb_place_goal.target_pose.position
                o = abb_place_goal.target_pose.orientation

                self.get_logger().info(
                    f"ABB PLACE GOAL -> Pos: [x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}] "
                    f"| Orient: [x={o.x:.3f}, y={o.y:.3f}, z={o.z:.3f}, w={o.w:.3f}]"
                )

                # abb_place_goal.target_pose.position.z = 0.24
                action_result = await self.send_action_goal(self.abb_client, abb_place_goal)
                self.get_logger().info(f'ABB Place Action Result: {action_result}')

                # if await self.check_and_recover(action_result, self.state):
                #     return
                self.state = "PROCESS_NEXT"
                
            # =========================
            # STATE 5: HANDOVER SEQUENCE
            # =========================
            elif self.state == "HANDOVER_SEQUENCE":
                self.state = "AR4_PICK_FOR_HANDOVER"

            elif self.state == "HANDOVER_EXECUTION":
                self.get_logger().info('Starting Handover...')

                goal = MoveToPose.Goal()
                goal.strategy = "GOTO_HANDOVER"
                action_result = await self.send_action_goal(self.ar4_point_client, goal)
                if await self.check_and_recover(action_result, self.state):
                    return
                # ABB Task Server handles Open -> Move -> Close sequence
                abb_goal = ExecuteTask.Goal()
                abb_goal.task_type = "PICK_FROM_HANDOVER"
                abb_goal.target_pose = self.handover_pose 
                action_result = await self.send_action_goal(self.abb_client, abb_goal)
                if await self.check_and_recover(action_result, self.state):
                    return
                # self.gripper.set_ar4_grip('OPEN')
                release_goal = MoveToPose.Goal()
                release_goal.strategy = "RELEASE"
                action_result = await self.send_action_goal(self.ar4_point_client, release_goal)
                if await self.check_and_recover(action_result, self.state):
                    return
                self.state = "PROCESS_NEXT"

            # =========================
            # STATE: RECOVERY
            # =========================
            elif self.state == "RECOVERY":
                self.get_logger().warn('Entering Recovery Mode: Moving AR4 to Home...')
                
                # 1. Prepare a "Home" or "Safe" command
                recovery_goal = MoveToPose.Goal()
                recovery_goal.strategy = "HOME" # Ensure your C++ commander handles a "HOME" strategy
                
                # 2. Attempt to move home
                action_result = await self.send_action_goal(self.ar4_point_client, recovery_goal)
                
                if action_result and action_result.success:
                    self.get_logger().info('Recovery Successful. Arm is home. Retrying current brick...')
                    # Option A: Retry the brick (put it back in the queue)
                    self.assembly_queue.insert(0, self.current_brick) 
                    self.state = "PROCESS_NEXT"
                else:
                    self.get_logger().error('Recovery Failed! Human intervention required.')
                    self.state = "EMERGENCY_STOP"

            elif self.state == "EMERGENCY_STOP":
                self.get_logger().error("SYSTEM HALTED. Please check for collisions or IK limits.")
                return # Stop the timer loop entirely

        except Exception as e:
            self.get_logger().error(f'State Machine Failed: {e}')

        if self.state != "DONE":
            self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)

    # async def send_action_goal(self, client, goal_msg):
    #     if not client.wait_for_server(timeout_sec=5.0):
    #         return False
    #     send_goal_future = client.send_goal_async(goal_msg)
    #     goal_handle = await send_goal_future
    #     if not goal_handle.accepted:
    #         return False
    #     result_future = goal_handle.get_result_async()
    #     result = await result_future
    #     return result.result
    
    async def send_action_goal(self, client, goal_msg):
        if not client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error(f'Action server {client._action_name} not available!')
            return None

        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected by {client._action_name}')
            return None

        result_future = goal_handle.get_result_async()
        result = await result_future

        # --- THE FEEDBACK CHECK ---
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return result.result
        else:
            self.get_logger().error(f'Action {client._action_name} failed with status: {result.status}')
            # return None
        
    # Create a small helper inside the state machine loop to save space
    async def check_and_recover(self, result, current_state_name):
        if result is None or not result.success:
            self.get_logger().error(f"Failure in {current_state_name}. Moving to RECOVERY.")
            self.state = "RECOVERY"
            return True # We need to recover
        return False # All good
    
    async def set_ar4_gripper(self, open_gripper: bool):
        """Helper to call the gripper service."""
        if not self.gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Gripper service not available!')
            return False
            
        req = SetBool.Request()
        req.data = open_gripper
        
        # Using await here works perfectly because of your ReentrantCallbackGroup
        result = await self.gripper_client.call_async(req)
        return result.success

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