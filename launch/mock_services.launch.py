#!/usr/bin/env python3
"""
Mock System Launch Configuration

This launch file starts all mock service providers:
- Mock Detection Node (for brick detection)
- Mock GUI Node (for assembly plan generation)
- Mock Grasping Pipeline Node (for grasp point calculation)

This allows testing the supervisor and other components without requiring:
- Real camera/vision system
- GUI interface
- Grasping ML model
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with all mock nodes"""
    
    return LaunchDescription([
        # Mock Detection Node - provides /detect_bricks service
        Node(
            package='supervisor_package',
            executable='mock_detection_node',
            name='mock_detection_node',
            output='screen',
            parameters=[]
        ),
        
        # Mock GUI Node - provides /get_assembly_plan service
        Node(
            package='supervisor_package',
            executable='mock_gui_node',
            name='mock_gui_node',
            output='screen',
            parameters=[]
        ),
        
        # Mock Grasping Pipeline Node - provides /grasp/get_grasp_point service
        Node(
            package='supervisor_package',
            executable='mock_grasping_node',
            name='mock_grasping_node',
            output='screen',
            parameters=[]
        ),
    ])
