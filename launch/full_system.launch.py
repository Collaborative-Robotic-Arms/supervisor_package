import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

def launch_setup(context, *args, **kwargs):
    # ---------------------------------------------------------
    # 1. Read Launch Arguments
    # ---------------------------------------------------------
    mode = LaunchConfiguration('mode').perform(context)
    # test_scenario = int(LaunchConfiguration('test_scenario').perform(context))
    
    is_sim = (mode == 'sim')
    
    # Load Kinematics (The "Math Book" for the robot)
    kinematics_config = load_yaml("dual_arms", "config/kinematics.yaml")
    if kinematics_config is None:
        print("WARNING: Could not find kinematics.yaml! Pose Commander might have planning issues.")

    nodes_to_start = []

    # ---------------------------------------------------------
    # 2. TASK SERVERS (Always Run)
    # ---------------------------------------------------------
    
    # AR4 Task Server
    nodes_to_start.append(Node(
        package='ar4_highlevel_bridge',
        executable='ar4_task_server', 
        name='ar4_task_server',
        output='screen',
        parameters=[
            {"use_sim": is_sim},
            {"use_sim_time": is_sim},
            {"robot_description_kinematics": kinematics_config}
        ]
    ))

    # ABB Task Server
    nodes_to_start.append(Node(
        package='abb_highlevel_bridge', 
        executable='abb_task_server',
        name='abb_task_server',
        output='screen',
        parameters=[
            {"use_sim": is_sim},
            {"use_sim_time": is_sim},
            {"robot_description_kinematics": kinematics_config} 
        ]
    ))

    # ---------------------------------------------------------
    # 3. HARDWARE-ONLY NODES
    # ---------------------------------------------------------
    if not is_sim:
        nodes_to_start.append(Node(
            package='ar4_highlevel_bridge',
            executable='gripper.py',
            name='ar4_gripper_server',
            output='screen',
            parameters=[{"use_sim_time": is_sim}]
        ))

    # ---------------------------------------------------------
    # 4. SIMULATION-ONLY NODES (The Mock Pipeline)
    # ---------------------------------------------------------
    # if is_sim:
    #     nodes_to_start.append(Node(
    #         package='supervisor_package',
    #         executable='mock_gui_node',
    #         name='mock_gui_node',
    #         output='screen',
    #         parameters=[{'test_scenario': test_scenario, 'use_sim_time': is_sim}]
    #     ))
        
    #     nodes_to_start.append(Node(
    #         package='supervisor_package',
    #         executable='mock_detection_node',
    #         name='mock_detection_node',
    #         output='screen',
    #         parameters=[{'test_scenario': test_scenario, 'use_sim_time': is_sim}]
    #     ))
        
    #     nodes_to_start.append(Node(
    #         package='supervisor_package',
    #         executable='mock_grasping_node',
    #         name='mock_grasping_node',
    #         output='screen',
    #         parameters=[{'test_scenario': test_scenario, 'use_sim_time': is_sim}]
    #     ))

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            choices=['sim', 'real'],
            description='Launch mode: sim (launches mocks) or real (hardware gripper only)'
        ),
        # DeclareLaunchArgument(
        #     'test_scenario',
        #     default_value='1',
        #     description='Test scenario (1=parallel safe, 2=parallel collision, 3=AR4->ABB, 4=ABB->AR4)'
        # ),
        OpaqueFunction(function=launch_setup)
    ])