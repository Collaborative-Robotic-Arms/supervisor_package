#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
# Essential TF2 imports
from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs

class TransformVisualizer(Node):
    def __init__(self):
        super().__init__('transform_debug_visualizer')
        
        # 1. Setup TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. Publisher for markers
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 20)
        
        # 3. Define Input Data
        # Input Grasp Points (Camera Frame) -> Will be RED
        self.gp1_cam = Pose()
        self.gp1_cam.position = Point(x=0.0, y=-0.3, z=0.22)
        self.gp1_cam.orientation.w = 1.0 # Valid quaternion required for Pose

        self.gp2_cam = Pose()
        self.gp2_cam.position = Point(x=0.0, y=0.0, z=0.14)
        self.gp2_cam.orientation.w = 1.0

        # Place Points (Base Frame) -> Will be BLUE
        self.pp1_base = Point(x=0.4, y=0.08, z=0.22)
        self.pp2_base = Point(x=0.7, y=0.2, z=0.14)
        
        # Timer to run the visualization loop periodically
        self.timer = self.create_timer(0.5, self.visualization_loop)
        self.get_logger().info("Transform Visualizer Running. Open RViz!")

    # =========================================
    # YOUR TRANSFORM FUNCTION (Slightly adapted for PoseStamped)
    # =========================================
    def transform_pose_to_abb(self, input_pose_stamped):
            # Ensure we have the necessary imports (should be at top of file)
            # from tf2_geometry_msgs import tf2_geometry_msgs
            
            target_frame = 'base_link'
            try:
                # The .transform() method automatically:
                # 1. Waits for the transform (if timeout is specified)
                # 2. Looks up the transform
                # 3. Applies the math to the pose
                # 4. Returns a new PoseStamped
                output_pose_stamped = self.tf_buffer.transform(
                    input_pose_stamped, 
                    target_frame, 
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                return output_pose_stamped

            except TransformException as ex:
                self.get_logger().error(f'TF2 Failure: {ex}')
                return None

    # =========================================
    # MARKER HELPER
    # =========================================
    def create_sphere(self, id, point, r, g, b, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "debug_shapes"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        # Size of the sphere (meters)
        marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05
        marker.color.r = float(r); marker.color.g = float(g); marker.color.b = float(b)
        marker.color.a = 0.8 # Slightly transparent
        marker.lifetime.sec = 1
        return marker

    # =========================================
    # MAIN LOOP
    # =========================================
    def visualization_loop(self):
        marker_id = 0
        camera_frame = 'camera'
        base_frame = 'base_link'

        # --- 1. Visualize BEFORE Transformation (RED) in Camera Frame ---
        self.publisher.publish(self.create_sphere(marker_id, self.gp1_cam.position, 1.0, 0.0, 0.0, camera_frame))
        marker_id += 1
        self.publisher.publish(self.create_sphere(marker_id, self.gp2_cam.position, 1.0, 0.0, 0.0, camera_frame))
        marker_id += 1


        # --- 2. PERFORM TRANSFORMATION ---
        # We must wrap the raw Pose into a PoseStamped so TF knows where it is coming FROM
        stamped_1 = PoseStamped(); stamped_1.header.frame_id = camera_frame; stamped_1.pose = self.gp1_cam
        stamped_2 = PoseStamped(); stamped_2.header.frame_id = camera_frame; stamped_2.pose = self.gp2_cam
        
        # Call your function
        transformed_1 = self.transform_pose_to_abb(stamped_1)
        transformed_2 = self.transform_pose_to_abb(stamped_2)


        # --- 3. Visualize AFTER Transformation (GREEN) in Base Frame ---
        if transformed_1 and transformed_2:
            # Success! Visualize the resulting positions
            self.publisher.publish(self.create_sphere(marker_id, transformed_1.pose.position, 0.0, 1.0, 0.0, base_frame))
            marker_id += 1
            self.publisher.publish(self.create_sphere(marker_id, transformed_2.pose.position, 0.0, 1.0, 0.0, base_frame))
            marker_id += 1
        else:
            # TF failed (usually just waiting for startup)
            pass


        # --- 4. Visualize PLACE Points (BLUE) in Base Frame ---
        self.publisher.publish(self.create_sphere(marker_id, self.pp1_base, 0.0, 0.0, 1.0, base_frame))
        marker_id += 1
        self.publisher.publish(self.create_sphere(marker_id, self.pp2_base, 0.0, 0.0, 1.0, base_frame))
        marker_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = TransformVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()