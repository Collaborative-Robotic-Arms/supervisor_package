import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Helper function to load YAML files (Standard ROS 2 MoveIt pattern)
def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return None

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. Configuration & Arguments
    # ---------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Load Kinematics (The "Math Book" for the robot)
    kinematics_config = load_yaml("dual_arms", "config/kinematics.yaml")
    if kinematics_config is None:
        print("WARNING: Could not find kinematics.yaml! Pose Commander might have planning issues.")

    # ---------------------------------------------------------
    # 2. Define Nodes
    # ---------------------------------------------------------
    
    # AR4 Point Control (The Muscle)
    ar4_task_server_node = Node(
        package='ar4_highlevel_bridge',
        executable='ar4_task_server', # Corrected executable name
        name='ar4_task_server',
        output='screen',
        parameters=[
            {"robot_description_kinematics": kinematics_config},
            {"use_sim": True},
            {"use_sim_time": use_sim_time}
        ]
    )

    # AR4 Middleman Controller
    ar4_controller = Node(
        package='supervisor_package',
        executable='ar4_controller_node',
        name='ar4_controller',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # ABB Middleman Controller
    abb_controller = Node(
        package='supervisor_package',
        executable='abb_controller_node',
        name='abb_controller',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )
    abb_task_server_node = Node(
        package='abb_highlevel_bridge', 
        executable='abb_task_server',
        name='abb_task_server',
        output='screen',
        parameters=[
            {"use_sim": True},
            {"use_sim_time": use_sim_time},
            {"robot_description_kinematics": kinematics_config} # <--- THIS FIXES THE CRASH
        ]
    )

    # ViSP IBVS (Visual Servoing)
    # visp_node = Node(
    #     package='perception_setup',
    #     executable='visp_ibvs_controller',
    #     name='visp_controller',
    #     output='screen',
    #     parameters=[{"use_sim_time": use_sim_time}]
    # )

    # Assembly Supervisor (The Brain)
    supervisor_node = Node(
        package='supervisor_package',
        executable='hybrid_supervisor_node', # Corrected from 'supervisor'
        name='system_supervisor',
        output='screen',
        parameters=[
            {"use_sim": True},
            {"use_sim_time": use_sim_time}
        ]
    )

    # ---------------------------------------------------------
    # 3. Return Launch Description
    # ---------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        ar4_task_server_node,
        # ar4_controller,
        # abb_controller,
        abb_task_server_node,
        # visp_node,
        supervisor_node
    ])