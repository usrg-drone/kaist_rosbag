# KAIST ROS bag of ROS2 Version
# !<CODE> Implies that it is replaced by below code

# !import rospy
import rclpy
import subprocess
import os
import signal

from rclpy.node import Node

from mavros_msgs.msg import State, RCIn
from std_msgs.msg import Bool

# !from nodelet.srv import NodeletUnload

import time


class RosbagRecord(Node):
    def __init__(self):
        # rosbag_record 이름의 node를 생성
        super().__init__('rosbag_record')

        self.declare_parameter('record_script', '')
        self.record_script = self.get_parameter('record_script').get_parameter_value().string_value

        self.declare_parameter('record_folder', '')
        self.record_folder = self.get_parameter('record_folder').get_parameter_value().string_value
        self.get_logger().info(f"Recording folder: {self.record_folder}")
        if self.record_script and self.record_folder:
            self.get_logger().info("record parameters received")
        else:
            self.get_logger().error(
                "no record script or folder specified."
            )
            rclpy.shutdown()
            return


        self.declare_parameter('trigger_topic_name', '/start_topic')
        self.trigger_topic_name = self.get_parameter('trigger_topic_name').value

        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').value

        self.declare_parameter('manager_name', 'nodelet_manager')
        self.manager_name = self.get_parameter('manager_name').value

        # RC input parameters
        self.declare_parameter('rc_channel', -1)
        self.rc_channel = self.get_parameter('rc_channel').value

        self.declare_parameter('rc_threshold', 1500)
        self.rc_threshold = self.get_parameter('rc_threshold').value

        self.get_logger().info(f"Listening to trigger topic: {self.trigger_topic_name}")
        self.get_logger().info(f"Recording script: {self.record_script}")

        # Create subscriber
        self.subscription = self.create_subscription(
            Bool,
            self.trigger_topic_name,
            self.trigger_subscriber_callback,
            10
        )

        # RC input subscriber (if channel is specified)
        if self.rc_channel >= 0:
            self.get_logger().info(
                "RC channel %d enabled with threshold %d"
                % (self.rc_channel, self.rc_threshold)
            )

            self.rc_subscription = self.create_subscription(
                RCIn,
                '/fcu/rc/in',
                self.rc_callback,
                10
            )
        else:
            self.get_logger().info("RC trigger disabled (rc_channel < 0)")

        # Timer (0.1 sec)
        self.sync_timer = self.create_timer(
            0.1,
            self.sync_timer_callback
        )

        # trigger states
        self.trigger = False
        self.last_trigger = False
        self.rc_button_pressed = False
        
        self.process = None  # Store the rosbag process


    def sync_timer_callback(self):
        os.system("sync")

        if not hasattr(self, "_sync_log_printed"):
            self.get_logger().info("Sync function is active!")
            self._sync_log_printed = True

# originally was --split --size=4096 --buffsize=4 --chunksize=256 --tcpnodelay
    def start_recording(self):
        if self.process is not None:
            self.get_logger().warn("Recording already running")
            return

        script_path = os.path.expanduser(self.record_script)
        if not os.path.exists(script_path):
            self.get_logger().error(f"Record script not found: {script_path}")
            return

        self.get_logger().info(f"Executing record script: {script_path}")

        self.process = subprocess.Popen(
            f"source {script_path} {os.path.expanduser(self.record_folder)}",
            shell=True,
            executable='/bin/bash'
        )
        self.trigger = True


    def stop_recording(self):
        """Stop ROS2 bag recording safely."""
        if self.process is None:
            return

        self.get_logger().info("Stopping rosbag recording")
        self.process.send_signal(signal.SIGINT) 
        self.process.wait()
        self.process = None
        self.trigger = False

    def update_trigger(self, new_trigger):
        """Common logic for updating trigger state and starting/stopping recording."""
        if new_trigger and not self.last_trigger:
            self.get_logger().info("Start rosbag")
            self.start_recording() 
        elif not new_trigger and self.last_trigger:
            self.get_logger().info("Stop rosbag")
            self.stop_recording()
        self.trigger = new_trigger
        self.last_trigger = self.trigger

    def trigger_subscriber_callback(self, msg: Bool):
        new_trigger = msg.data
        if self.verbose:
            self.get_logger().info(f"Trigger topic update: {new_trigger}")
        self.update_trigger(new_trigger)


        
    def rc_callback(self, msg: RCIn):
        """Handle RC input as toggle button. Rising edge (value goes above threshold) toggles recording state."""
        if self.rc_channel < 0 or self.rc_channel >= len(msg.channels):
            return

        channel_value = msg.channels[self.rc_channel]
        button_pressed = channel_value > self.rc_threshold

        # Detect rising edge (button press)
        if button_pressed and not self.rc_button_pressed:
            # Toggle recording state
            new_trigger = not self.trigger
            self.get_logger().info(f"RC button pressed! Toggling recording: {self.trigger} -> {new_trigger}")
            self.update_trigger(new_trigger)

        self.rc_button_pressed = button_pressed

        if self.verbose:
            self.get_logger().info(
                f"RC channel {self.rc_channel} value: {channel_value}, button_pressed: {button_pressed}"
            )

def main(args=None):

    rclpy.init(args=args)

    node = RosbagRecord()

    node.get_logger().info(f"{node.get_name()} start")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, stopping recording...")
    finally:
        if node.process is not None:
            node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
