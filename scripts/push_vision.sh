#!/bin/bash
if [ "$1" != "" ] && [ "$2" != "" ]; then
	hostname=$1
    pass=$2

    blimp_vision_dir="/home/$USER/ros2_ws/src/blimp_vision"
	
	# Check if device is online
	timeout=1
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
        # Stop opi_vision service
        echo $pass | ssh -tt opi@$hostname "sudo systemctl stop opi_vision"

        # Verify workspace folder
		echo ">>Verifying workspace"
        ssh opi@$hostname "mkdir -p /home/opi/ros2_ws/src"

        # Copy blimp_vision package
        echo ">>Copying blimp_vision to opi@$hostname"
        scp -r $blimp_vision_dir opi@$hostname:/home/opi/ros2_ws/src/

        # Colcon build
        echo ">>Executing colcon build"
        ssh opi@$hostname "source /opt/ros/humble/setup.bash; cd /home/opi/ros2_ws/; colcon build"

        # Start opi_vision service
        echo $pass | ssh -tt opi@$hostname "sudo systemctl start opi_vision"

		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: push_vision.sh [hostname] [opi password]"
fi