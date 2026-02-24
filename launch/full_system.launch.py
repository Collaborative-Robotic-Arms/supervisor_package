import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Helper function to load YAML files (Standard ROS 2 MoveIt pattern)
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    
    # ---------------------------------------------------------
    # 1. Load Kinematics (The "Math Book" for the robot)
    # ---------------------------------------------------------
    # We load the kinematics.yaml file into a dictionary
    kinematics_config = load_yaml("annin_ar4_moveit_config", "config/kinematics.yaml")

    # Guard check: Print a warning if file is not found
    if kinematics_config is None:
        print("WARNING: Could not find kinematics.yaml! Pose Commander will fail.")

    # ---------------------------------------------------------
    # 2. Define Nodes
    # ---------------------------------------------------------
    
    # Node 1: Point Control (Pose Commander)
    # We pass the loaded kinematics dictionary to the parameters list
    pose_commander_node = Node(
        package='point_control_pkg',
        executable='pose_commander',
        name='ar4_pose_commander',
        output='screen',
        parameters=[
            {"robot_description_kinematics": kinematics_config}
        ]
    )

    # Node 2: ViSP IBVS (Visual Servoing)
    visp_node = Node(
        package='perception_setup',
        executable='visp_ibvs_controller',
        name='visp_controller',
        output='screen'
    )

    # Node 3: Supervisor (State Machine)
    supervisor_node = Node(
        package='supervisor_package',
        executable='supervisor',
        name='system_supervisor',
        output='screen'
    )

    # ---------------------------------------------------------
    # 3. Return Launch Description
    # ---------------------------------------------------------
    return LaunchDescription([
        pose_commander_node,
        visp_node,
        supervisor_node
    ])