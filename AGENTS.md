# kaist_rosbag — Agent Notes

Single-node ROS 2 package. `rosbag_recorder` spawns `ros2 bag record -s mcap`
as a subprocess and drives its start/stop from a YAML-selected trigger.

## Files

- `kaist_rosbag/rosbag_recorder.py` — the whole node: config load, command
  build, trigger callbacks, subprocess start/stop, status publishing.
- `launch/rosbag_recorder.launch.py` — declares the `config_file` argument
  (default `handcarry.yaml`) and launches the node with it as a parameter.
- `config/` — `handcarry.yaml`, `arming.yaml`, `rc.yaml` (profiles) and
  `mcap_qos.yaml` (shared MCAP storage settings).

## Facts that must stay true

- `config_file` is the only ROS parameter; everything else is in the YAML and is
  resolved from the installed `config/` directory.
- Trigger modes are exactly `handcarry`, `arm_state`, `rc` — selected by
  `trigger.mode`. Unknown modes raise on load (`TriggerMode(...)`).
- `handcarry` records immediately; `arm_state` records while armed; `rc` records
  while `channels[rc_trigger_channel] > rc_trigger_threshold`.
- `rc_callback` guards the channel index: if it is negative or beyond the
  message's `channels` length, it warns and ignores the message.
- RC defaults: channel `5`, threshold `1700` — keep them config-overridable.
- Output path is `$LOG_DIR/<save_dir>/<YYMMDD_HHMMSS>`, `LOG_DIR` defaulting to
  `~/bags`.
- Topics with a leading `/` are recorded verbatim; others are namespace-prefixed.
- Status is `std_msgs/Bool` on `/recording_status` at 2 Hz.
- Stop = `SIGINT` to the recorder process group, then `SIGKILL` on timeout — the
  recorder runs in its own session (`start_new_session=True`).

