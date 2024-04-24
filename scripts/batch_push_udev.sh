#!/bin/bash

# USED TO PUSH udev

if [ "$1" != "" ]; then
	pass=$1

    blimp_vision_dir="/home/laptop-4/ros2_ws/src/blimp_vision"

    # Iterate through all opi's
    for n in {1..6};
    do
        hostname="192.168.0.10$n"
        

        # Check if device is online
        timeout=1
        ping $hostname -c 1 -W $timeout > /dev/null
        if [ $? == 0 ]; then
            echo ">>Pushing udev to opi$n"

            # Verify folder
            ssh opi@$hostname "mkdir -p /home/opi/udev"
            # Copy udev from laptop to opi temporary directory
            scp -r $blimp_vision_dir/scripts/udev opi@$hostname:/home/opi/
            # Copy udev on opi from temporary directory to /etc/sudev/rules.d
            echo $pass | ssh -tt opi@$hostname "sudo cp /home/opi/udev/00-teensy.rules /etc/udev/rules.d/; sudo cp /home/opi/udev/99-elp-stereo-sync-camera.rules /etc/udev/rules.d/"
            # Restart udev rules
            echo $pass | ssh -tt opi@$hostname "sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger"

            echo ">>Done."
        else
            echo "opi$n is offline :("
        fi
    done
else
	echo "Usage: batch_push_systemd.sh [opi password]"
fi



