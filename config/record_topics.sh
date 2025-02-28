#!/bin/bash
source /opt/ros/galactic/setup.bash
source ~/glim_ws/install/setup.bash
echo "Starting rosbag record..."

DIR_NAME=$(date "+%Y-%m-%d_%H-%M-%S")
OUTPUT_DIR=~/glim_ws/bags/$DIR_NAME

echo "Saving bag file in: $OUTPUT_DIR"
ros2 bag record -o "$OUTPUT_DIR" \
/mavros/state \
/livox/lidar \
/livox/imu \
/glim_ros/aligned_points \
/glim_ros/map \
/glim_ros/odom \
/glim_ros/points \
/glim_ros/pose \
/mavros/local_position/accel \
/mavros/local_position/odom \
/mavros/local_position/pose \
/mavros/imu/data \
/mavros/setpoint_velocity/cmd_vel \
/mavros/local_position/velocity_local \
/mavros/setpoint_raw/attitude \
/mavros/setpoint_raw/global \
/mavros/setpoint_raw/local \
/mavros/setpoint_raw/target_attitude \
/mavros/setpoint_raw/target_global \
/mavros/setpoint_raw/target_local





