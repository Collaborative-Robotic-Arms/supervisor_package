#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose, Point

# Import the specific messages the SERVICES expect
from supervisor_package.srv import GetAssemblyPlan
from supervisor_package.msg import SuperBrick  # <-- This is the one that was crashing

# Import these for the other services
from dual_arms_msgs.srv import GetGrasp, DetectBricks
from dual_arms_msgs.msg import Brick, GraspPoint


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts euler angles (radians) to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy # x
    q[1] = cr * sp * cy + sr * cp * sy # y
    q[2] = cr * cp * sy - sr * sp * cy # z
    q[3] = cr * cp * cy + sr * sp * sy # w
    return q
    
# --- DEFINE YOUR ANGLES HERE (in degrees) ---
roll = 180.0
pitch = 0.0 
yaw = 0

# Convert to Radians
r_rad = math.radians(roll)
p_rad = math.radians(pitch)
y_rad = math.radians(yaw)

# Convert to Quaternion
q = euler_to_quaternion(r_rad, p_rad, y_rad)

class MockRobotSystem(Node):
    def __init__(self):
        super().__init__('mock_robot_system')

        self.plan_srv = self.create_service(GetAssemblyPlan, 'get_assembly_plan', self.get_plan_callback)
        self.detect_srv = self.create_service(DetectBricks, 'detect_bricks', self.detect_bricks_callback)
        self.grasp_srv = self.create_service(GetGrasp, 'grasp/get_grasp_point', self.get_grasp_callback)


        self.get_logger().info('Mock System Ready. Testing AR4 sequence...')


    def get_plan_callback(self, request, response):
            self.get_logger().info('Mock: Sending Assembly Plan...')
            
            plan = []
            
            brick = SuperBrick()
            brick.id = 1
            brick.type = "I_BRICK"      # Must be a string based on your .msg
            brick.start_side = "AR4"    # Must be a string based on your .msg
            brick.target_side = "GRID"  # Must be a string based on your .msg
            
            # Initialize the poses so they aren't null
            brick.pickup_pose = Pose()
            brick.pickup_pose.position.x = 0.5
            brick.pickup_pose.orientation.w = 1.0
            
            brick.place_pose = Pose()
            brick.place_pose.position.x = 0.7
            brick.place_pose.position.y = 0.2
            brick.place_pose.position.z = 0.14
            brick.place_pose.orientation.x = q[0]
            brick.place_pose.orientation.y = q[1]
            brick.place_pose.orientation.z = q[2]
            brick.place_pose.orientation.w = q[3]
            
            plan.append(brick)

            brick2 = SuperBrick()
            brick2.id = 2
            brick2.type = "T_BRICK"    
            brick2.start_side = "AR4"  
            brick2.target_side = "GRID"
            
            # Initialize the poses so they aren't null
            brick2.pickup_pose = Pose()
            brick2.pickup_pose.position.x = 0.57
            brick2.pickup_pose.orientation.w = 1.0
            
            brick2.place_pose = Pose()
            brick2.place_pose.position.x = 0.65
            brick2.place_pose.position.y = -0.15
            brick2.place_pose.position.z = 0.14
            brick2.place_pose.orientation.x = q[0]
            brick2.place_pose.orientation.y = q[1]
            brick2.place_pose.orientation.z = q[2]
            brick2.place_pose.orientation.w = q[3]
            plan.append(brick2)
            
            response.plan = plan
            return response

    def detect_bricks_callback(self, request, response):
            self.get_logger().info('Mock: Sending Detected Bricks (IDs 1 & 2)...')
            
            # --- Brick 1 ---
            brick1 = Brick()
            brick1.id = 1
            brick1.pose.position.x = 0.5
            brick1.pose.position.z = 0.14
            brick1.pose.orientation.w = 1.0
            
            # --- Brick 2 ---
            brick2 = Brick()
            brick2.id = 2
            brick2.pose.position.x = 0.57 
            brick2.pose.position.z = 0.14
            brick2.pose.orientation.w = 1.0

            # Return both in the list
            response.bricks = [brick1, brick2]
            
            # Handover pose (default)
            response.handover_pose = Pose()
            response.handover_pose.orientation.w = 1.0
            
            return response

    def get_grasp_callback(self, request, response):
            self.get_logger().info(f'Mock: Processing Grasp Request for Brick ID: {request.brick_index}')
            
            response.success = True
            gp = GraspPoint()

            # Logic to return different points based on the ID requested
            if request.brick_index == "1":
                self.get_logger().info("Mock: Providing Grasp Point for Brick 1")
                gp.pose.position = Point(x=0.0, y=0.0, z=0.14)
                gp.pose.orientation.x = 0.0
                gp.pose.orientation.y = 0.0
                gp.pose.orientation.z = 0.0
                gp.pose.orientation.w = 1.0

            elif request.brick_index == "2":
                self.get_logger().info("Mock: Providing Grasp Point for Brick 2")
                gp.pose.position = Point(x=-0.15, y=0.1, z=0.14)
                # Using your manual orientation for ABB/Brick 2
                gp.pose.orientation.x = 0.0
                gp.pose.orientation.y = 0.0
                gp.pose.orientation.z = 0.0
                gp.pose.orientation.w = 1.0

            else:
                self.get_logger().warn(f"Mock: Brick ID {request.brick_index} not recognized!")
                response.success = False

            response.grasp_point = gp
            return response

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotSystem()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()