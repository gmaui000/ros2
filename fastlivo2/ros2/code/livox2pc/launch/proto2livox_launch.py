from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare output format argument
    output_format_arg = DeclareLaunchArgument(
        'output_format',
        default_value='livox',
        description='Output format: livox (for livox_ros_driver::msg::CustomMsg) or std (for sensor_msgs::msg::PointCloud2)'
    )

    return LaunchDescription([
        output_format_arg,
        Node(
            package='livox2pc',
            executable='proto2livox',
            name='proto2livox',
            output='screen',
            parameters=[{
                'output_format': LaunchConfiguration('output_format')
            }],
            remappings=[
                # Add any topic remappings here if needed
            ]
        )
    ])
