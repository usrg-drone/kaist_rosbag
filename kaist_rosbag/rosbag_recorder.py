import copy
import datetime
import os
import signal
import subprocess

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('rosbag_recorder')
        self.node_name = self.get_name()

        self.declare_parameter('config_file', 'test.yaml')
        config_file = self.get_parameter(
            'config_file').get_parameter_value().string_value
        self.config_path = os.path.join(
            get_package_share_directory('kaist_rosbag'), 'config', config_file)
        if not os.path.exists(self.config_path):
            self.get_logger().error(f"Config not found: {self.config_path}")
            raise FileNotFoundError(self.config_path)

        self.declare_parameter(
            'save_dir', os.path.join(os.environ['HOME'], 'bags'))
        self.save_dir = self.get_parameter(
            'save_dir').get_parameter_value().string_value
        os.makedirs(self.save_dir, exist_ok=True)
        os.chdir(self.save_dir)

        self.declare_parameter('trigger_topic_name', '/mavros/state')
        self.trigger_topic_name = self.get_parameter(
            'trigger_topic_name').get_parameter_value().string_value

        self.mcap_qos_dir = os.path.join(
            get_package_share_directory('kaist_rosbag'), 'config')

        with open(self.config_path) as f:
            self.cfg = yaml.safe_load(f)

        self.command_prefix = ["ros2", "bag", "record", "-s", "mcap"]
        self.command = None
        self.build_command()

        self.process = None
        self.last_trigger = False
        self.last_manual_trigger = False

        self.state_subscription = self.create_subscription(
            State, self.trigger_topic_name, self.trigger_callback, 10)
        self.manual_trigger_subscription = self.create_subscription(
            Bool, '/record_trigger', self.manual_trigger_callback, 10)

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10)
        self.status_pub = self.create_publisher(
            Bool, f"{self.node_name}/bag_recording_status", reliable_qos)
        self.create_timer(0.5, self.pub_status_callback)

        self.get_logger().info(f"Config: {self.config_path}")
        self.get_logger().info(f"Save dir: {self.save_dir}")
        self.get_logger().info(f"Trigger topic: {self.trigger_topic_name}")

    def is_recording(self):
        return self.process is not None and self.process.poll() is None

    def build_command(self):
        namespace = self.get_namespace()
        cfg = self.cfg

        cmd = list(self.command_prefix)
        cmd.extend(str(arg) for arg in cfg.get('args', []))

        if cfg.get('mcap_qos'):
            qos_path = os.path.join(self.mcap_qos_dir, str(cfg['mcap_qos']))
            cmd.extend(['--storage-config-file', qos_path])

        suffix = []
        if 'exclude' in cfg:
            if 'topics' in cfg:
                self.get_logger().error('Cannot mix exclude with topics.')
                raise ValueError('exclude+topics together')
            suffix.append('--all')
            for topic in cfg['exclude']:
                suffix.extend(['--exclude', topic])
        else:
            for topic in cfg['topics']:
                full_topic = (
                    topic if topic.startswith('/') else f"{namespace}/{topic}")
                suffix.append(full_topic)

        self.command = {'prefix': cmd, 'suffix': suffix}
        self.get_logger().info(
            f"CMD: {' '.join(cmd)} ... ({len(suffix)} topic args)")

    def pub_status_callback(self):
        msg = Bool()
        msg.data = self.is_recording()
        self.status_pub.publish(msg)

    def manual_trigger_callback(self, msg):
        if msg.data and not self.last_manual_trigger:
            self.get_logger().info("Manual trigger: Start recording")
            self.start_recording()
        elif not msg.data and self.last_manual_trigger:
            self.get_logger().info("Manual trigger: Stop recording")
            self.stop_recording()
        self.last_manual_trigger = msg.data

    def trigger_callback(self, msg):
        trigger = msg.armed
        if trigger and not self.last_trigger:
            self.get_logger().info("Armed: Start recording")
            self.start_recording()
        elif not trigger and self.last_trigger:
            self.get_logger().info("Disarmed: Stop recording")
            self.stop_recording()
        self.last_trigger = trigger

    def start_recording(self):
        if self.is_recording():
            self.get_logger().info("Already recording, ignore request")
            return

        time_suffix = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        cmd = copy.deepcopy(self.command['prefix'])
        cmd.extend(['-o', time_suffix])
        if self.command['suffix']:
            cmd.extend(self.command['suffix'])

        self.get_logger().info(f"Running: {' '.join(cmd)}")
        try:
            self.process = subprocess.Popen(cmd, start_new_session=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to start: {exc}")
            return
        self.get_logger().info(
            f"Started pid={self.process.pid} -> {time_suffix}")

    def stop_recording(self):
        if not self.is_recording():
            self.process = None
            return

        self.get_logger().info(f"Stopping pid={self.process.pid}")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=10.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            self.get_logger().warning("SIGINT timeout, SIGKILL")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait(timeout=5.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self.get_logger().error("did not terminate")
        finally:
            self.process = None


def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
