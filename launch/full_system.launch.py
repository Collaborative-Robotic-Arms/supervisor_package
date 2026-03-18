import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# --- NEW IMPORT REQUIRED FOR CONDITIONAL LAUNCHING ---
from launch.conditions import UnlessCondition

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
    test_scenario = LaunchConfiguration('test_scenario')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_test_scenario = DeclareLaunchArgument(
        'test_scenario',
        default_value='1',
        description='Test scenario (1=parallel no collision, 2=parallel with collision, 3=AR4→ABB handover, 4=ABB→AR4 handover)'
    )

    # ---------------------------------------------------------
    # MoveIt Dual Arms Launch (Spawns arms in Gazebo)
    # ---------------------------------------------------------

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
        executable='ar4_task_server', 
        name='ar4_task_server',
        output='screen',
        parameters=[
            {"use_sim": use_sim_time},
            {"robot_description_kinematics": kinematics_config},
            {"use_sim_time": use_sim_time}
        ]
    )

    # AR4 Gripper (Only runs on Hardware)
    ar4_gripper_server_node = Node(
        package='ar4_highlevel_bridge',
        executable='gripper.py',
        name='ar4_gripper_server',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_sim_time) 
    )

    
    abb_task_server_node = Node(
        package='abb_highlevel_bridge', 
        executable='abb_task_server',
        name='abb_task_server',
        output='screen',
        parameters=[
            {"use_sim": use_sim_time},
            {"use_sim_time": use_sim_time},
            {"robot_description_kinematics": kinematics_config} 
        ]
    )

    # Assembly Supervisor (The Brain)
    supervisor_node = Node(
        package='supervisor_package',
        executable='supervisor_node', 
        name='system_supervisor',
        output='screen',
        parameters=[
            {"use_sim": use_sim_time},
            {"use_sim_time": use_sim_time}
        ]
    )

    # ---------------------------------------------------------
    # 3. Return Launch Description
    # ---------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_test_scenario,
        
        ar4_task_server_node,
        abb_task_server_node,
        ar4_gripper_server_node,
        
        # supervisor_node
    ])