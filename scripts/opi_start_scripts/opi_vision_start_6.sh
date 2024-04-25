# !/bin/bash
source /home/opi/ros2_ws/install/setup.bash
ros2 launch blimp_vision elp_opi.launch.py namespace:='SuperBeefBlimp' camera_id:='camera9'