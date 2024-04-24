# Read Me for blimp_vision
**This directory should be placed in /home/$USER/ros2_ws/src/**

## One-Time Setup Scripts

### udev
1. In `blimp_vision/scripts`, run the following batch command:
    ```
    ./batch_push_udev.sh [opi password]
    ```

### systemd services (for microros and opi_vision)
1. In `blimp_vision/scripts`, run the following batch command:
    ```
    ./batch_push_systemd.sh [opi password]
    ```

## Configure Blimp Namespace and Camera for opi#
1. Open the appropriate opi_vision_start script located at:
`blimp_vision/scripts/opi_vision_start_scripts/opi_vision_start_#.sh`
2. Edit appropriate namespace and camera parameters.
    - If needed, you can change the video device by adding to the opi_vision_start script (example):
        - `camera_device:='/dev/video0'`
3. Once blimp parameters are set, run the following batch command:
    ```
    ./batch_push_opi_vision_start.sh [opi password]
    ```

## Push and Build Vision Code
1. To push the vision code to a specific blimp, run the following command:
    ```
    ./push_vision.sh [hostname] [opi password]
    ```