import datetime
import os
import signal
import subprocess
import yaml
from enum import Enum

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from mavros_msgs.msg import RCIn, State
from std_msgs.msg import Bool


class TriggerMode(Enum):
    HANDCARRY = "handcarry"
    RC = "rc"
    ARM_STATE = "arm_state"


class RosbagRecorder(Node):
    def __init__(self):
        super().__init__("rosbag_recorder")

        self.declare_parameter("config_file", "INVALID_FILE")
        config_file = self.get_parameter("config_file").value
        self.config_path = os.path.join(
            get_package_share_directory("kaist_rosbag"), "config", config_file
        )
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Cannot find config file: {self.config_path}")

        self.load_recorder_config()
        self.build_command()

        print(
            "\n".join(
                [
                    "*" * 50,
                    "Rosbag recorder",
                    f"  config  : {os.path.basename(self.config_path)}",
                    f"  save dir: {self.save_dir}",
                    f"  trigger : {self.trigger_mode.value}",
                    "  topics  :",
                    *[f"    - {topic}" for topic in self.record_topics],
                    "*" * 50,
                ]
            ),
            flush=True,
        )

        self.process = None
        self.last_trigger = False

        self.trigger_subscription = None

        if self.trigger_mode == TriggerMode.ARM_STATE:
            self.trigger_subscription = self.create_subscription(
                State, self.arm_state_topic, self.arm_state_callback, 1
            )
        elif self.trigger_mode == TriggerMode.RC:
            self.trigger_subscription = self.create_subscription(
                RCIn, self.rc_topic, self.rc_callback, 1
            )
        else:  # handcarry / fallback
            self.start_recording()

        self.status_pub = self.create_publisher(Bool, "/recording_status", 1)
        self.create_timer(0.5, self.pub_status_callback)

    def load_recorder_config(self):
        with open(self.config_path) as f:
            self.cfg = yaml.safe_load(f) or {}

        trigger_cfg = self.cfg.get("trigger")
        self.trigger_mode = TriggerMode(
            trigger_cfg.get("mode", TriggerMode.ARM_STATE.value)
        )

        if self.trigger_mode == TriggerMode.ARM_STATE:
            self.arm_state_topic = trigger_cfg.get("arm_state_topic", "/mavros/state")
        elif self.trigger_mode == TriggerMode.RC:
            self.rc_topic = trigger_cfg.get("rc_topic", "/mavros/rc/in")
            self.rc_trigger_channel = trigger_cfg.get("rc_trigger_channel", 5)
            self.rc_trigger_threshold = trigger_cfg.get("rc_trigger_threshold", 1700)

        self.save_dir = os.path.join(
            os.environ.get("LOG_DIR", os.path.expanduser("~/bags")),
            str(self.cfg.get("save_dir", "")),
        )
        os.makedirs(self.save_dir, exist_ok=True)

    def build_command(self):
        self.base_command_prefix = ["ros2", "bag", "record", "-s", "mcap"]
        self.record_command_prefix = []
        self.record_topics = []

        namespace = self.get_namespace().rstrip("/")
        cfg = self.cfg

        cmd = list(self.base_command_prefix)
        cmd.extend(str(arg) for arg in cfg.get("args", []))

        if cfg.get("mcap_qos"):
            qos_path = os.path.join(
                get_package_share_directory("kaist_rosbag"),
                "config",
                str(cfg["mcap_qos"]),
            )
            cmd.extend(["--storage-config-file", qos_path])

        topics = []
        for topic in cfg.get("topics"):
            topic = str(topic)
            if topic.startswith("/"):
                topics.append(topic)
            elif namespace:
                topics.append(f"{namespace}/{topic}")
            else:
                topics.append(f"/{topic}")

        self.record_command_prefix = cmd
        self.record_topics = topics

    def is_recording(self):
        return self.process is not None and self.process.poll() is None

    def pub_status_callback(self):
        msg = Bool()
        msg.data = self.is_recording()
        self.status_pub.publish(msg)
        # Popen can succeed even when ros2 bag record exits immediately.
        if self.process is not None and self.process.poll() is not None:
            returncode = self.process.returncode
            if returncode != 0:
                self.get_logger().error(
                    f"bag record exited unexpectedly rc={returncode}"
                )
            self.process = None

    def rc_callback(self, msg):
        """Start or stop recording on RC switch rising/falling edges."""

        if self.rc_trigger_channel < 0 or len(msg.channels) <= self.rc_trigger_channel:
            self.get_logger().warning(
                "RC channel index "
                f"{self.rc_trigger_channel} is unavailable in message with "
                f"{len(msg.channels)} channels; ignoring RC trigger messages"
            )
            return

        trigger = msg.channels[self.rc_trigger_channel] > self.rc_trigger_threshold

        # Only trigger on edges so repeated RC messages do not restart recording.
        if trigger and not self.last_trigger:
            self.get_logger().info("RC trigger: Start recording")
            self.start_recording()
        elif not trigger and self.last_trigger:
            self.get_logger().info("RC trigger: Stop recording")
            self.stop_recording()
        self.last_trigger = trigger

    def arm_state_callback(self, msg):
        trigger = msg.armed

        if trigger and not self.last_trigger:  # disarmed -> armed
            self.get_logger().info("Armed: Start recording")
            self.start_recording()
        elif not trigger and self.last_trigger:  # armed -> disarmed
            self.get_logger().info("Disarmed: Stop recording")
            self.stop_recording()
        self.last_trigger = trigger

    def start_recording(self):
        if self.is_recording():
            self.get_logger().info("Recording process already running, ignore request")
            return

        output_path = os.path.join(
            self.save_dir, datetime.datetime.now().strftime("%y%m%d_%H%M%S")
        )
        cmd = list(self.record_command_prefix)
        cmd.extend(["-o", output_path])
        if self.record_topics:
            cmd.extend(self.record_topics)

        self.get_logger().info(f"Running: {' '.join(cmd)}")
        try:
            # Start a new session so stop_recording can signal the whole group.
            self.process = subprocess.Popen(cmd, start_new_session=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to start: {exc}")
            return
        self.get_logger().info(f"Started pid={self.process.pid} -> {output_path}")

    def stop_recording(self):
        if not self.is_recording():
            self.get_logger().warning("No existing recording process, ignore request")
            self.process = None
            return

        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=10.0)
            self.get_logger().info("Recording process killed")
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            self.get_logger().warning("SIGINT timeout, SIGKILL")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait(timeout=5.0)
                self.get_logger().info("Recording process killed")
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self.get_logger().error("Failed to terminate recording process")
        finally:
            self.process = None


def main(args=None):

    rclpy.init(args=args)
    node = RosbagRecorder()

    def request_shutdown(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_shutdown)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, request_shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
