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
            description='Namespace for the node'
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='camera1',
            description='Name of the calibration file'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace=[LaunchConfiguration('namespace'), '/sync'],
            parameters=[os.path.join(get_package_share_directory('blimp_vision'), 'param', 'elp_laptop.yaml')],
            output='screen'
        ),
        Node(
            package='opencv_telemetry',
            executable='split_sync_images',
            namespace=LaunchConfiguration('namespace'),
            name='split_sync_image_node',
            parameters=[
                {'calibration_file': LaunchConfiguration('calibration_file')},
            ],
        ),
        ExecuteProcess(
	        cmd=[[
	            'ros2 run camera_calibration cameracalibrator ',
	            '--size 8x6 --square 0.06 ',
	            'left:=', 			LaunchConfiguration('namespace'), '/left/image_raw ',
	            'right:=', 			LaunchConfiguration('namespace'), '/right/image_raw ',
	            'left_camera:=', 	LaunchConfiguration('namespace'), '/left ',
	            'right_camera:=', 	LaunchConfiguration('namespace'), '/right'
	        ]],
	        shell=True
    	)
  ])
