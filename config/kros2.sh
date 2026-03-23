#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="$1/$(date +%Y%m%d_%H%M%S)"

ros2 bag record \
--storage mcap \
--max-cache-size 16384 \
--max-bag-size 524288000 \
--qos-profile-overrides-path "$SCRIPT_DIR/qos_override_mavros.yaml" \
-o "$OUTPUT" \
/camera/color/image_raw \
/camera/camera_info \
/tf \
/tf_static \
/clock \
/diagnostics \
/rosout \
/mavros/vision_pose/pose \
/fcu/imu/data_raw \
/fcu/imu/data_filtered \
/fcu/attitude \
/fcu/rc_override \
/fcu/rc/in \
/fcu/motor \
/fcu/battery \
/fcu/esc_telemetry \
/mavros/local_position/odom \
/odometry \
/mavros/local_position/odom_ekf \
/mars_ov/odom_state_out \
/vo_drift \
/vo_input \
/user_cmd/start \
/user_cmd/goto \
/reference/active_reference/path \
/reference/active_setpoint/path \
/reference/fixed_setpoint/path \
/mavros/setpoint_raw/attitude \
/race_track \
/ov_msckf/pathimu \
/ov_msckf/points_msckf \
/ov_msckf/points_slam \
/ov_msckf/loop_feats \
/ov_msckf/points_sim \
/ov_msckf/odomimu \
/ov_msckf/poseimu \
/ov_msckf/odomimu_aligned \
/camera/odom/sample_aligned \
/gate_detection/detections \
/gate_detection/detections_matched \
/gate_detection/pose_cov \
/gate_detection/pose_error_cov \
/gate_detection/poses \
/gate/reprojections \
/odom_check \
/drone_pose \
/race_track/fake \
/vrpn_client_node/drone_gt/pose \
/setpoints/markers \
/setpoints/path \
/mars/odom_state_out \
/mars/full_state_lite_out \
/mars/full_state_out \
/debug/command \
/mpc_prediction/path \
/sampled_setpoints/path \
/global_reference/path \
/debug/drone_pose \
/mpc/reference \
/user_cmd/race_start \
/reference/odom \
/debug/control_error
