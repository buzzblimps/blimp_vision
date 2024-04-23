#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	# Check if device is online
	timeout=2
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		echo ">>Verifying workspace"
        ssh opi@$hostname "mkdir -p ~/ros2_ws/src"

        echo ">>Copying blimp_vision to opi@$hostname"
        scp -r ../blimp_vision opi@$hostname:/home/opi/ros2_ws/src/

        echo ">>Executing colcon build"
        ssh opi@$hostname "source /opt/ros/humble/setup.bash; cd ~/ros2_ws/; colcon build"

		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: copy_to_opi.sh [hostname]"
fi