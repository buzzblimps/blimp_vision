# Read Me for blimp_vision
- **This directory should be placed in /home/$USER/ros2_ws/src/**
- All scripts are located in [blimp_vision/scripts](/scripts/)

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

### microros start scripts
1. Run the appropriate command:
    ### Single Command
    ```
    ./push_microros_start.sh [opi number] [opi password]
    ```
    ### Batch Command
    ```
    ./batch_push_microros_start.sh [opi password]
    ```

## Configure Blimp Namespace and Camera for opi#
1. Open the appropriate opi_vision_start script located at:
`blimp_vision/scripts/opi_vision_start_scripts/opi_vision_start_#.sh`

2. Edit appropriate namespace and camera parameters.
    - If needed, you can change the video device by adding to the opi_vision_start script (example):
        - `camera_device:='/dev/video0'`

3. Once blimp parameters are set, run the appropriate command:
    ### Single Command
    ```
    ./push_vision_start.sh [opi number] [opi password]
    ```
    ### Batch Command
    ```
    ./batch_push_vision_start.sh [opi password]
    ```

## Push and Build Vision Code (slow)
1. To push the vision code to a specific blimp, run the following command:
    ```
    ./push_vision.sh [hostname] [opi password]
    ```

## Check status of services
### Vision
1. Run the following command:
```
./check_vision_service.sh [hostname]
```

### Microros
1. Run the following command:
```
./check_microros_service.sh [hostname]
```