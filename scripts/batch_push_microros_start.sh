#!/bin/bash

blimp_vision_dir="/home/$USER/ros2_ws/src/blimp_vision"

if [ "$1" != "" ]; then
    pass=$1

	# Iterate through all opi's
	for n in {1..6};
	do
		hostname="192.168.0.10$n"

		# Check if device is online
		timeout=1
		ping $hostname -c 1 -W $timeout > /dev/null
		if [ $? == 0 ]; then
			echo ">>Pushing microros_start.sh to opi$n"

			# Stop opi_vision service
			echo $pass | ssh -tt opi@$hostname "sudo systemctl stop microros"

			ssh opi@$hostname "mkdir -p /home/opi/scripts"
			scp $blimp_vision_dir/scripts/opi_start_scripts/microros_start.sh opi@$hostname:/home/opi/scripts/
			ssh opi@$hostname "chmod 755 /home/opi/scripts/microros_start.sh"
			
			# Start opi_vision service
			echo $pass | ssh -tt opi@$hostname "sudo systemctl start microros"
			
			echo ">>Done."
		else
			echo "opi$n is offline :("
		fi
	done

else
	echo "Usage: batch_push_microros_start.sh [opi password]"
fi

