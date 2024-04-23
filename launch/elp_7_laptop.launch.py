import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='MaryO',
            description='Namespace for the node(s)'
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='camera7',
            description='Camera ID'
        ),
        DeclareLaunchArgument(
            'imshow',
            default_value='False',
            description='Use imshow to render image(s)'
        ),
        DeclareLaunchArgument(
            'depth_rate',
            default_value='1.0',
            description='Frequency (in Hz) of depth estimation'
        ),
        DeclareLaunchArgument(
            'save_video',
            default_value='False',
            description='Save video feed'
        ),
        Node(
            package='blimp_vision',
            executable='blimp_vision_node',
            name='blimp_vision_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                os.path.join(get_package_share_directory('blimp_vision'), 'param', 'elp_config_laptop.yaml'),
                {'camera_id': LaunchConfiguration('camera_id')},
                {'imshow': LaunchConfiguration('imshow')},
                {'depth_rate': LaunchConfiguration('depth_rate')},
                {'save_video': LaunchConfiguration('save_video')}
            ],
            output='screen'
        )
    ])
