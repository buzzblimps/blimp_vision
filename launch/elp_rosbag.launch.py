import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/willie/Documents/rosbags/green_ball.db3'],
            output='screen'
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='camera7',
            description='Camera ID'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[os.path.join(get_package_share_directory('blimp_vision'), 'param', 'elp_config_laptop.yaml')],
            output='screen'
        ),
        # Node(
        #     package='blimp_vision',
        #     executable='blimp_vision_node',
        #     name='blimp_vision_node',
        #     namespace=LaunchConfiguration('namespace'),
        #     parameters=[
        #         os.path.join(get_package_share_directory('blimp_vision'), 'param', 'elp_config_laptop.yaml'),
        #         {'camera_id': LaunchConfiguration('camera_id')}
        #     ],
        #     output='screen'
        # )
    ])
