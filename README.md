
# ROS2 Automated Rosbag Recording System

## Features
- Automatic rosbag recording using `ros2 bag record`
- Support for MCAP storage format
- Trigger-based start/stop mechanism
- RC channel toggle support (rising-edge detection)
- Configurable recording topics via shell script
- Automatic timestamp-based output directory generation
- Periodic disk sync for improved data integrity

## Directory Structure
- `config/kros2.sh` : Bash script defining rosbag recording behavior and topic list
- `launch/kros2.launch.py` : Launch file for starting the recording node
- `kros/kros2.py` : Core ROS2 node handling triggers and process control

## Prerequisites
- ROS2 Humble or later
- `ros2 bag` with MCAP support
- MAVROS (if using flight controller topics)
- Properly sourced ROS2 workspace:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash

## Build Instructions
From the root of the ROS2 workspace:

```bash  
cd ~/ros2_ws  
colcon build  
source install/setup.bash
```

## Usage
1. Launch the Recording Node
ros2 launch kros kros2.launch.py

2. Recording Behavior
Recording is controlled by one of the following methods:
    Topic Trigger (Default: /start_topic)
        True → Start recording
        False → Stop recording

    RC Trigger (Default: Channel 9)
        Recording is toggled when the RC channel value exceeds the threshold (default: 1500).
        Each rising edge switches the state between start and stop.

3. Output Directory
Recorded data is stored in the following format:
~/bags/<timestamp>/
    Example: /home/user/bags/20260317_234201/

Each directory contains:
    metadata.yaml
    .mcap bag files

4. Customization
    Change Recording Directory:
    ros2 launch kros kros2.launch.py record_folder:=~/my_bags

    Change Trigger Topic:
    ros2 launch kros kros2.launch.py topic:=/my_trigger

    Modify Recorded Topics:
    Edit the config/kros2.sh file to update the list of topics.
    
    ***You don't need to re-build the package, but you have to modify ~/ros2_ws/install/kros/share/kros/config/kros2.sh file***

    Adjust RC Settings:
    Modify the parameters in kros2.launch.py:
    Python

    'rc_channel': 9,
    'rc_threshold': 1500

## Important Notes

 - Directory Conflict: The output directory must not exist prior to
   recording. The system automatically generates a unique
   timestamp-based folder.
 - Script Deployment: The script executed at runtime is located in the
   install/ directory. 
 - After modifying config/kros2.sh, you must rebuild the workspace
   (colcon build) or manually copy the script to the install    path.
 - Data Safety: Periodic disk synchronization is performed to reduce
   data loss in case of unexpected power failure.

 Troubleshooting
 - Output folder already exists: Ensure the recording script does not pre-create the output directory.
 -  Script changes not applied: Rebuild the workspace and source the setup file again.
 -  Recording does not start:
        Verify that the trigger topic is being published.
        Check the RC channel configuration and values.
        Confirm the validity of the script path.

 - Termination
Recording stops automatically when:
    The trigger condition becomes False.
    The RC toggle is triggered again.
    The node is terminated (e.g., Ctrl+C).
