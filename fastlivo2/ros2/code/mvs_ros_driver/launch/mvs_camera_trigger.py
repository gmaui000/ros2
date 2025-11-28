from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('mvs_ros_driver')
    
    # 定义启动参数，控制是否启动 rviz2，默认关闭
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch rviz2'
    )
    
    camera_node = Node(
        package='mvs_ros_driver',
        executable='grabImgWithTrigger',
        name='mvs_camera_trigger',
        arguments=[os.path.join(package_dir, 'config', 'left_camera_trigger.yaml')],
        respawn=True,
        output='screen'
        # Note: launch-prefix for debugging can be added as:
        # prefix='gdb -ex run --args'
    )
    
    # 定义 rviz2 节点，使用条件启动
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(package_dir, 'rviz_cfg', 'mvs_camera.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_rviz_arg,
        camera_node,
        rviz_node
    ])
