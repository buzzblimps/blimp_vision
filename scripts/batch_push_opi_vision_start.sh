#!/bin/bash

blimp_vision_dir="/home/laptop-4/ros2_ws/src/blimp_vision"

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
			echo ">>Pushing opi_vision_start.sh to opi$n"

			# Stop opi_vision service
			echo $pass | ssh -tt opi@$hostname "sudo systemctl stop opi_vision"

			ssh opi@$hostname "mkdir -p /home/opi/scripts"
			scp $blimp_vision_dir/scripts/opi_vision_start_scripts/opi_vision_start_$n.sh opi@$hostname:/home/opi/scripts/
			ssh opi@$hostname "mv /home/opi/scripts/opi_vision_start_$n.sh /home/opi/scripts/opi_vision_start.sh; chmod 755 /home/opi/scripts/opi_vision_start.sh"
			
			# Start opi_vision service
			echo $pass | ssh -tt opi@$hostname "sudo systemctl start opi_vision"
			
			echo ">>Done."
		else
			echo "opi$n is offline :("
		fi
	done

else
	echo "Usage: batch_push_opi_vision_start.sh [opi password]"
fi

