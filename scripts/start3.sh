#!/bin/bash
source ~/ros2_ws/install/setup.bash
ros2 launch blimp_vision elp_opi.launch.py namespace:='TurboBlimp' camera_id:='camera3'