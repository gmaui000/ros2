from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('mvs_ros_driver')
    
    left_camera_node = Node(
        package='mvs_ros_driver',
        executable='grabImgWithTrigger',
        name='camera',
        namespace='left_camera',
        arguments=[os.path.join(package_dir, 'config', 'left_camera_trigger.yaml')],
        respawn=True,
        output='screen'
    )
    
    right_camera_node = Node(
        package='mvs_ros_driver',
        executable='grabImgWithTrigger',
        name='camera',
        namespace='right_camera',
        arguments=[os.path.join(package_dir, 'config', 'right_camera_trigger.yaml')],
        respawn=True,
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(package_dir, 'rviz_cfg', 'mvs_camera.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        left_camera_node,
        right_camera_node,
        rviz_node
    ])
