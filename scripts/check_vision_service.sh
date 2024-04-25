#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1

	# Check if device is online
	timeout=1
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then

        # Check service status
        ssh opi@$hostname "systemctl status opi_vision"
        
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: check_vision_service.sh [hostname]"
fi