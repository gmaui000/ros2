#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # Get package share directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_file = os.path.join(pkg_dir, 'config', 'nodes.yaml')
    
    # Launch arguments for configuration file
    config_path_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to supervisor configuration file'
    )
    
    # Launch arguments for node enable/disable overrides
    # These parameters have higher priority than nodes.yaml configuration
    fast_livo_enable_arg = DeclareLaunchArgument(
        'fast_livo_enabled',
        default_value='',
        description='Override fast_livo node enabled state (true/false). Empty means use config file.'
    )
    
    foxglove_enable_arg = DeclareLaunchArgument(
        'foxglove_enabled',
        default_value='',
        description='Override foxglove node enabled state (true/false). Empty means use config file.'
    )
    
    livox_enable_arg = DeclareLaunchArgument(
        'livox_enabled',
        default_value='',
        description='Override livox node enabled state (true/false). Empty means use config file.'
    )
    
    mvs_enable_arg = DeclareLaunchArgument(
        'mvs_enabled',
        default_value='',
        description='Override mvs node enabled state (true/false). Empty means use config file.'
    )
    
    calibration_enable_arg = DeclareLaunchArgument(
        'calibration_enabled',
        default_value='',
        description='Override calibration node enabled state (true/false). Empty means use config file.'
    )
    
    # Supervisor node with all parameters
    supervisor_node = Node(
        package='supervisor',
        executable='supervisor',
        name='supervisor',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            {'fast_livo_enabled': LaunchConfiguration('fast_livo_enabled')},
            {'foxglove_enabled': LaunchConfiguration('foxglove_enabled')},
            {'livox_enabled': LaunchConfiguration('livox_enabled')},
            {'mvs_enabled': LaunchConfiguration('mvs_enabled')},
            {'calibration_enabled': LaunchConfiguration('calibration_enabled')},
        ],
        remappings=[
            ('/ui/command', '/supervisor/command'),
            ('/ui/status', '/supervisor/status')
        ]
    )
    
    return LaunchDescription([
        config_path_arg,
        fast_livo_enable_arg,
        foxglove_enable_arg,
        livox_enable_arg,
        mvs_enable_arg,
        calibration_enable_arg,
        supervisor_node
    ])
