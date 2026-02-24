from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo

def generate_launch_description():

    # 1. Mock AR4 Node (Simulates Gripper/Robot)
    mock_ar4_node = Node(
        package='supervisor_package',
        executable='mock_ar4',
        name='mock_ar4',
        output='screen'
    )

    # 2. Supervisor Node (The Brain)
    supervisor_node = Node(
        package='supervisor_package',
        executable='supervisor_node',
        name='supervisor',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg=">>> Starting Mock Environment & TF..."),
        mock_ar4_node,

        # Wait 3 seconds to ensure TF is available before starting the Supervisor
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg=">>> TF Ready. Starting Supervisor..."),
                supervisor_node
            ]
        )
    ])