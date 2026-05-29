import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

import subprocess
from mavros_msgs.msg import State
from std_msgs.msg import Bool

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
            State,
            self.trigger_topic_name,
            self.trigger_callback,
            10
        )

        self.manual_trigger_subscription = self.create_subscription(
            Bool,
            '/record_trigger',
            self.manual_trigger_callback,
            10
        )

        self.manual_trigger_publisher = self.create_publisher(Bool, '/record_trigger',10)
        self.publish_manual_trigger()
        self.timer = self.create_timer(1.0, self.publish_manual_trigger)
        self.trigger = False
        self.manual_trigger_state = False
        self.last_manual_trigger_state = self.manual_trigger_state

        
        self.last_trigger = self.trigger
        
        self.process = None  # Store the rosbag process

    def start_recording(self):
        """Start ROS 2 bag recording by executing the shell script."""
        self.get_logger().info(f"Executing: {self.record_script}")
        self.process = subprocess.Popen(
            ["bash", self.record_script],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

    def stop_recording(self):
        """Stop ROS 2 bag recording."""
        if self.process:
            self.get_logger().info("Stopping rosbag recording...")
            self.process.terminate()
            self.process.wait()
            self.process = None

    def publish_manual_trigger(self):
    
        msg = Bool()
        msg.data = False
        self.manual_trigger_publisher.publish(msg)

    def manual_trigger_callback(self,msg):
        self.manual_trigger_state = msg.data
        self.evaluate_manual()

    def trigger_callback(self, msg):
        """Handle the trigger topic messages."""
        self.trigger = msg.armed  # Start recording if the drone is armed

        if self.trigger and not self.last_trigger:
            self.get_logger().info("Trigger detected: Start recording")
            self.start_recording()
            # self.stop_recording()
        elif not self.trigger and self.last_trigger:
            self.get_logger().info("Trigger released: Stop recording")
            # self.start_recording()
            self.stop_recording()

        # if self.manual_trigger_state and not self.last_manual_trigger_state:
        #     self.get_logger().info("Trigger detected: Start recording")
        #     self.start_recording()
        #     # self.stop_recording()
        # elif not self.manual_trigger_state and self.last_manual_trigger_state:
        #     self.get_logger().info("Trigger released: Stop recording")
        #     # self.start_recording()
        #     self.stop_recording()
        self.last_trigger = self.trigger
        # self.last_manual_trigger_state = self.manual_trigger_state
    
    def evaluate_manual(self):
        if self.manual_trigger_state and not self.last_manual_trigger_state:
            self.get_logger().info("Trigger detected: Start recording")
            self.start_recording()
            # self.stop_recording()
        elif not self.manual_trigger_state and self.last_manual_trigger_state:
            self.get_logger().info("Trigger released: Stop recording")
            # # self.start_recording()
            # self.stop_recording()
        self.last_manual_trigger_state = self.manual_trigger_state

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
