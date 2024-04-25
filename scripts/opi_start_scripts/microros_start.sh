#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/opi/microros_ws/install/setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0