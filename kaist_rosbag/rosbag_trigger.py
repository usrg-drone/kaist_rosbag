import os
import signal
import subprocess
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mavros_msgs.msg import State

class RosbagRecord(Node):
    def __init__(self):
        super().__init__('rosbag_record')

        # Declare and get launch parameters
        self.declare_parameter('record_script', 'record_topics.sh')
        script_name = self.get_parameter('record_script').get_parameter_value().string_value
        self.record_script = os.path.join(
            get_package_share_directory('kaist_rosbag'), 
            'config', 
            script_name
        )
        if not os.path.exists(self.record_script):
            self.get_logger().error(f"Record script not found: {self.record_script}")
            raise FileNotFoundError(self.record_script)

        self.declare_parameter('trigger_topic_name', '/mavros/state')
        self.trigger_topic_name = self.get_parameter('trigger_topic_name').get_parameter_value().string_value

        self.get_logger().info(f"Listening to trigger topic: {self.trigger_topic_name}")
        self.get_logger().info(f"Recording script: {self.record_script}")

        # Create subscriber
        self.subscription = self.create_subscription(
            State, self.trigger_topic_name, self.trigger_callback, 10)

        self.manual_trigger_subscription = self.create_subscription(
            Bool, '/record_trigger', self.manual_trigger_callback, 10)

        self.last_trigger = False
        self.last_manual_trigger = False
        self.process = None  # Store the rosbag process

    def is_recording(self):
        return self.process is not None and self.process.poll() is None

    def start_recording(self):
        if self.is_recording():
            self.get_logger().info("Recording process is already running, ignore request")
            return

        self.get_logger().info(f"Executing: {self.record_script}")
        self.process = subprocess.Popen(
            ["bash", self.record_script],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True
        )

    def stop_recording(self):
        """Stop ROS 2 bag recording."""
        if not self.is_recording():
            self.get_logger().info("Recording process is not running, ignore request")
            self.process = None
            return

        self.get_logger().info("Stopping rosbag recording...")
        process = self.process
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            process.wait(timeout=10.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                process.wait(timeout=5.0)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                self.get_logger().error("rosbag did not terminate after SIGTERM")
            self.process = None

    def manual_trigger_callback(self,msg):
        if msg.data and not self.last_manual_trigger:
            self.get_logger().info("Manual trigger: Start recording")
            self.start_recording()
        elif not msg.data and self.last_manual_trigger:
            self.get_logger().info("Manual trigger: Stop recording")
            self.stop_recording()
        self.last_manual_trigger = msg.data

    def trigger_callback(self, msg):
        self.trigger = msg.armed  # Start recording if the drone is armed

        if self.trigger and not self.last_trigger:
            self.get_logger().info("Armed: Start recording")
            self.start_recording()
        elif not self.trigger and self.last_trigger:
            self.get_logger().info("Disarmed: Stop recording")
            self.stop_recording()
        self.last_trigger = self.trigger
    
def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
