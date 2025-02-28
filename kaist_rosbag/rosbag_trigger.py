import rclpy
import subprocess
import os
from rclpy.node import Node
from mavros_msgs.msg import State
from std_msgs.msg import Bool
import time


class RosbagRecord(Node):
    def __init__(self):
        super().__init__('rosbag_record')

        # Declare and get launch parameters
        self.declare_parameter('record_script', '~/glim_ws/src/kaist_rosbag/config/record_topics.sh')
        self.record_script = self.get_parameter('record_script').get_parameter_value().string_value

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


        

        self.manual_trigger = self.create_subscription(
            Bool,
            '/record_trigger',
            self.manual_trigger_callback,
            10
        )

        self.manual_trigger_publisher = self.create_publisher(Bool, '/record_trigger',10)
        self.publish_manual_trigger()
        self.timer = self.create_timer(1.0, self.publish_manual_trigger)
        self.trigger = False
        self.manual_trigger = False
        self.last_manual_trigger = self.manual_trigger

        
        self.last_trigger = self.trigger
        
        self.process = None  # Store the rosbag process

    def start_recording(self):
        """Start ROS 2 bag recording by executing the shell script."""
        if not os.path.exists(os.path.expanduser(self.record_script)):
            self.get_logger().error("Record script not found!")
            return

        self.get_logger().info(f"Executing: {self.record_script}")
        self.process = subprocess.Popen(
            ["bash", os.path.expanduser(self.record_script)],
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
        # self.get_logger().info(f"Publishing attempt {self.publish_attempts + 1}: False to /record_trigger")
        


    def manual_trigger_callback(self,msg):
        self.manual_trigger = msg.data
        self.evaluate_manual()

    def trigger_callback(self, msg):
        """Handle the trigger topic messages."""
        self.trigger = msg.armed  # Start recording if the drone is armed
        # self.trigger = msg.mode

        if self.trigger and not self.last_trigger:
            self.get_logger().info("Trigger detected: Start recording")
            self.start_recording()
            # self.stop_recording()
        elif not self.trigger and self.last_trigger:
            self.get_logger().info("Trigger released: Stop recording")
            # self.start_recording()
            self.stop_recording()
        # if self.trigger == "POSCTL" and self.last_trigger == "STABILIZED":
        #     self.get_logger().info("Trigger detected: Start recording")
        #     self.start_recording()
        #     # self.stop_recording()
        # elif self.trigger == "STABILIZED" and self.last_trigger == "POSCTL":
        #     self.get_logger().info("Trigger released: Stop recording")
        #     # self.start_recording()
        #     self.stop_recording()

        # if self.manual_trigger and not self.last_manual_trigger:
        #     self.get_logger().info("Trigger detected: Start recording")
        #     self.start_recording()
        #     # self.stop_recording()
        # elif not self.manual_trigger and self.last_manual_trigger:
        #     self.get_logger().info("Trigger released: Stop recording")
        #     # self.start_recording()
        #     self.stop_recording()
        self.last_trigger = self.trigger
        # self.last_manual_trigger = self.manual_trigger
    
    def evaluate_manual(self):
        if self.manual_trigger and not self.last_manual_trigger:
            self.get_logger().info("Trigger detected: Start recording")
            self.start_recording()
            # self.stop_recording()
        elif not self.manual_trigger and self.last_manual_trigger:
            self.get_logger().info("Trigger released: Stop recording")
            # # self.start_recording()
            # self.stop_recording()
        self.last_manual_trigger = self.manual_trigger




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
