from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Mock system provides GUI service and ABB action (already present)
    mock_system = Node(
        package='supervisor_package',
        executable='mock_system',
        name='mock_system'
    )

    # Mock AR4 action servers
    mock_ar4 = Node(
        package='supervisor_package',
        executable='mock_ar4',
        name='mock_ar4'
    )

    # Supervisor node
    supervisor = Node(
        package='supervisor_package',
        executable='supervisor',
        name='assembly_supervisor'
    )

    ld.add_action(mock_system)
    ld.add_action(mock_ar4)
    ld.add_action(supervisor)

    return ld
