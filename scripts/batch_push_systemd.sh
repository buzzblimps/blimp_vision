#!/bin/bash

# USED TO PUSH systemd SERVICES

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
            echo ">>Pushing systemd services to opi$n"

            # Stop services
            echo $pass | ssh -tt opi@$hostname "sudo systemctl stop opi_vision; sudo systemctl stop microros"
            # Copy service from laptop to opi temporary directory
            scp -r $blimp_vision_dir/scripts/systemd opi@$hostname:/home/opi/
            # Copy services on opi from temporary directory to /etc/systemd/system
            # Then, reload daemon
            # Then, enable services
            # Then, start services
            echo $pass | ssh -tt opi@$hostname "sudo cp /home/opi/systemd/opi_vision.service /etc/systemd/system/; sudo cp /home/opi/systemd/microros.service /etc/systemd/system/; sudo systemctl enable opi_vision; sudo systemctl enable microros; sudo systemctl daemon-reload; sudo systemctl start opi_vision; sudo systemctl start microros"

            echo ">>Done."
        else
            echo "opi$n is offline :("
        fi
    done
else
	echo "Usage: batch_push_systemd.sh [opi password]"
fi



