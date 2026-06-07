#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import time
import json
import math

class MasterDataLogger(Node):
    def __init__(self):
        super().__init__('master_data_logger')
        self.get_logger().info('Master Data Logger Initialized. Waiting for GLOBAL_START...')
        
        # Subscribers
        self.create_subscription(String, '/task_events', self.task_callback, 10)
        self.create_subscription(String, '/zone_status', self.zone_callback, 10)
        
        # TF Listener for Distance
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.record_distance) # 10Hz logging
        
        # Global Time
        self.start_time = None
        
        # Data Containers
        self.data = {
            "total_time": 0.0,
            "separation_times": [],
            "separation_distances": [],
            "ar4_utilization": {"active": 0.0, "idle": 0.0},
            "abb_utilization": {"active": 0.0, "idle": 0.0},
            "ar4_zones": [],
            "abb_zones": []
        }
        
        # State Trackers
        self.ar4_active_start = None
        self.abb_active_start = None
        
        self.ar4_current_zone = "SAFE"
        self.abb_current_zone = "SAFE"
        self.ar4_zone_start = 0.0
        self.abb_zone_start = 0.0

    def get_elapsed(self):
        if self.start_time is None: return 0.0
        return time.time() - self.start_time

    def task_callback(self, msg):
        event = msg.data
        t = time.time()
        
        if event == "GLOBAL_START" and self.start_time is None:
            self.start_time = t
            self.get_logger().info("⏱️ RECORDING STARTED!")
            
        elif self.start_time is not None:
            if event == "AR4_START":
                self.ar4_active_start = t
            elif event == "AR4_END" and self.ar4_active_start:
                self.data["ar4_utilization"]["active"] += (t - self.ar4_active_start)
                self.ar4_active_start = None
                
            elif event == "ABB_START":
                self.abb_active_start = t
            elif event == "ABB_END" and self.abb_active_start:
                self.data["abb_utilization"]["active"] += (t - self.abb_active_start)
                self.abb_active_start = None

    def zone_callback(self, msg):
        if self.start_time is None: return
        
        # Parse the string: "AR4: SAFE | ABB: APPROACH"
        parts = msg.data.split("|")
        if len(parts) == 2:
            ar4_new_zone = parts[0].split(":")[1].strip()
            abb_new_zone = parts[1].split(":")[1].strip()
            
            elapsed = self.get_elapsed()
            
            # AR4 Zone Change
            if ar4_new_zone != self.ar4_current_zone:
                duration = elapsed - self.ar4_zone_start
                self.data["ar4_zones"].append((self.ar4_zone_start, duration, self.ar4_current_zone))
                self.ar4_current_zone = ar4_new_zone
                self.ar4_zone_start = elapsed
                
            # ABB Zone Change
            if abb_new_zone != self.abb_current_zone:
                duration = elapsed - self.abb_zone_start
                self.data["abb_zones"].append((self.abb_zone_start, duration, self.abb_current_zone))
                self.abb_current_zone = abb_new_zone
                self.abb_zone_start = elapsed

    def record_distance(self):
        if self.start_time is None: return
        try:
            trans = self.tf_buffer.lookup_transform('tool0', 'ar4_ee_link', rclpy.time.Time())
            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            dz = trans.transform.translation.z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            
            self.data["separation_times"].append(self.get_elapsed())
            self.data["separation_distances"].append(dist)
        except Exception:
            pass # Ignore if TF is missing for a frame

    def save_data(self):
        if self.start_time is None: return
        
        # Finalize open durations
        elapsed = self.get_elapsed()
        self.data["total_time"] = elapsed
        
        self.data["ar4_zones"].append((self.ar4_zone_start, elapsed - self.ar4_zone_start, self.ar4_current_zone))
        self.data["abb_zones"].append((self.abb_zone_start, elapsed - self.abb_zone_start, self.abb_current_zone))
        
        self.data["ar4_utilization"]["idle"] = elapsed - self.data["ar4_utilization"]["active"]
        self.data["abb_utilization"]["idle"] = elapsed - self.data["abb_utilization"]["active"]

        try:
            with open("/home/kareem-saleh/collab_ws/src/supervisor_package/supervisor_package/thesis_data2.json", "w") as f:
                json.dump(self.data, f)
            self.get_logger().info("💾 ALL DATA SAVED TO 'thesis_data2.json'")
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MasterDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Intercepted Ctrl+C! Saving data before shutting down...")
        node.save_data()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()