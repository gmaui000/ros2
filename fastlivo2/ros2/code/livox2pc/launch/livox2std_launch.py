from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    livox_topic = DeclareLaunchArgument(
        'livox_topic',
        default_value='/livox/lidar',
        description='Livox LiDAR topic name'
    )
    
    rosbag_path = DeclareLaunchArgument(
        'rosbag_path',
        default_value='',
        description='Path to input rosbag file (leave empty to use live data)'
    )
    
    # 启动livox2pc节点
    livox2std = Node(
        package='livox2pc',
        executable='livox2std',  # 确保ROS 2版本的可执行文件名称正确
        name='livox2std',
        output='screen',
        parameters=[{
            'livox_topic': LaunchConfiguration('livox_topic'),
            'rosbag_path': LaunchConfiguration('rosbag_path'),
        }],
        remappings=[
            # 根据需要添加话题重映射
            # ('input', LaunchConfiguration('livox_topic')),
            # ('output', '/point_cloud')
        ]
    )
    
    return LaunchDescription([
        livox_topic,
        rosbag_path,
        livox2std
    ])
