# kaist_rosbag

A ROS 2 node (`rosbag_recorder`) that runs `ros2 bag record -s mcap` as a child process and starts/stops it from a trigger. What gets recorded, where, and how it is triggered are all read from a single YAML config in `config/`.

## How it works

On startup the node:

1. Reads the `config_file` parameter, resolves it under the package's installed
   `config/` directory.
2. Loads the YAML and builds the `ros2 bag record` command line from it.
3. Picks a trigger from `trigger.mode`:
   - `handcarry` — starts recording immediately and keeps recording until the
     node shuts down.
   - `arm_state` — subscribes to a `mavros_msgs/State` topic; starts on
     disarm→arm, stops on arm→disarm.
   - `rc` — subscribes to a `mavros_msgs/RCIn` topic; starts/stops on the
     rising/falling edge of `channels[channel] > threshold`. 
4. Publishes a `std_msgs/Bool` on `/recording_status` every 0.5 s — `true` while
   the recorder process is alive.

Recording is stopped by sending `SIGINT` to the recorder's process group (so the
MCAP file is closed cleanly), escalating to `SIGKILL` after a 10 s timeout. On
node shutdown (Ctrl-C, `SIGTERM`, `SIGHUP`) any active recording is stopped
first.

## Output location

Each recording is written to:

```
$LOG_DIR/<save_dir>/<YYMMDD_HHMMSS>
```

`LOG_DIR` defaults to `~/bags` if unset. `save_dir` comes from the config. The
timestamp is generated when recording starts.

## Build & run

```bash
colcon build --packages-select kaist_rosbag
source install/setup.bash

ros2 launch kaist_rosbag rosbag_recorder.launch.py config_file:=handcarry.yaml
```

## Config format

| Key | Used when | Meaning |
| --- | --- | --- |
| `trigger.mode` | always | `handcarry`, `arm_state`, or `rc`. |
| `trigger.arm_state_topic` | `arm_state` | State topic (default `/mavros/state`). |
| `trigger.rc_topic` | `rc` | RC topic (default `/mavros/rc/in`). |
| `trigger.rc_trigger_channel` | `rc` | Channel index into `RCIn.channels` (default `5`). |
| `trigger.rc_trigger_threshold` | `rc` | PWM value the channel must exceed (default `1700`). |
| `save_dir` | always | Sub-folder name under `$LOG_DIR`. |
| `mcap_qos` | optional | Filename in `config/` passed to `--storage-config-file`. |
| `args` | optional | Extra arguments inserted into the `ros2 bag record` line. |
| `topics` | always | List of topics to record. |

Topics that start with `/` are recorded as-is. Topics without a leading `/` are
prefixed with the node's namespace (or just `/` when there is no namespace).

### Example (`arming.yaml`)

```yaml
trigger:
  mode: "arm_state"
  arm_state_topic: "/mavros/state"

save_dir: "arm_state"

mcap_qos: "mcap_qos.yaml"

args:
  - -b
  - 1000000000          # ~1GB per file
  - --max-cache-size
  - 50000000            # 50MB buffer (memory saving)

topics:
  - /recording_status
  - /mavros/state
  - /mavros/rc/in
  ...
```

`mcap_qos.yaml` holds the MCAP storage settings (compression / chunk size) and
is shared by all profiles.
