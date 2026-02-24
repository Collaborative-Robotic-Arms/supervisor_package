from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    # --- 1. LOCATE OTHER LAUNCH FILES ---
    # We find the paths to the launch files you currently run manually
    # moveit_launch_path = PathJoinSubstitution([
    #     FindPackageShare('dual_arms'), 'launch', 'moveit_dualarms.launch.py'
    # ])

    # abb_server_launch_path = PathJoinSubstitution([
    #     FindPackageShare('abb_highlevel_bridge'), 'launch', 'server_abb.launch.py'
    # ])

    # --- 2. DEFINE INFRASTRUCTURE (MoveIt & Drivers) ---
    
    # Launch MoveIt (The heaviest process)
    # launch_moveit = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(moveit_launch_path)
    # )

    # Launch ABB Driver Server
    # launch_abb_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(abb_server_launch_path)
    # )

    # Launch AR4 C++ Action Server (Pose)
    node_ar4_pose = Node(
        package='dual_arms',
        executable='ar4_pose_server',
        name='ar4_pose_server',
        output='screen'
    )

    # Launch AR4 Gripper Service
    node_ar4_gripper = Node(
        package='dual_arms',
        executable='ar4_gripper_service',
        name='ar4_gripper_service',
        output='screen'
    )

    # --- 3. DEFINE CONTROLLERS (Python Adapters) ---
    # These act as the bridge between Supervisor and Action Servers
    
    # node_abb_controller = Node(
    #     package='supervisor_package',
    #     executable='abb_controller_node',
    #     name='abb_controller',
    #     output='screen'
    # )

    node_ar4_controller = Node(
        package='supervisor_package',
        executable='ar4_controller_node',
        name='ar4_controller',
        output='screen'
    )

    return LaunchDescription([
        # A. Start Infrastructure Immediately
        LogInfo(msg=">>> PHASE 1: Starting MoveIt and Drivers..."),
        # launch_moveit,
        # launch_abb_server,
        node_ar4_pose,
        node_ar4_gripper,

        # B. Start Controllers after 5 seconds
        # (Gives time for Action Servers to advertise themselves)
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg=">>> PHASE 2: Starting Robot Controllers..."),
                # node_abb_controller, 
                node_ar4_controller
            ]
        )
    ])