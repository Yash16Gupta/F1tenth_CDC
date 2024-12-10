import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class MuxNode(Node):
    def __init__(self):
        super().__init__('mux_node')

        # Topics for Node A and Node B
        self.node_a_topic = '/alpp_cmd'
        self.node_b_topic = '/ftg_cmd'
        self.drive_topic = '/drive'
        self.lidar_topic = '/scan'

        # Publishers and Subscribers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.node_a_subscriber = self.create_subscription(
            AckermannDriveStamped, self.node_a_topic, self.node_a_callback, 10)
        self.node_b_subscriber = self.create_subscription(
            AckermannDriveStamped, self.node_b_topic, self.node_b_callback, 10)

        self.lidar_subscriber = self.create_subscription(
            LaserScan, self.lidar_topic, self.lidar_callback, 10)

        # State
        self.use_node_b = False
        self.last_msg_a = None
        self.last_msg_b = None
        self.transitioning_to_a = False  # To track if transitioning to Node A

        # Constants
        self.safe_distance = 1.0
        self.detection_angle = 45
        self.instance_threshold = 1
        self.transition_delay = 2.0  # Delay in seconds

        self.transition_timer = None

    def lidar_callback(self, scan_msg):
        """Processes LiDAR data to check for obstacles."""
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min

        obstacle_detected = self.detect_obstacle(ranges, angle_increment, angle_min)

        if obstacle_detected:
            if self.transition_timer:  # Cancel any ongoing transition to Node A
                self.transition_timer.cancel()
                self.transition_timer = None
                self.transitioning_to_a = False
            self.get_logger().info("Obstacle detected! Switching to Node B.")
            self.use_node_b = True
        else:
            if not self.transitioning_to_a and self.use_node_b:
                self.get_logger().info("No obstacle detected. Starting transition to Node A.")
                self.transitioning_to_a = True
                self.transition_timer = self.create_timer(
                    self.transition_delay, self.complete_transition_to_a)

        self.publish_drive_command()

    def detect_obstacle(self, ranges, angle_increment, angle_min):
        """Detect obstacles within a specified field of view and safe distance."""
        center_idx = len(ranges) // 2
        fov_indices = int(self.detection_angle / 2 / (angle_increment * 180 / np.pi))

        left_idx = max(0, center_idx - fov_indices)
        right_idx = min(len(ranges), center_idx + fov_indices)

        count = 0
        for i in range(left_idx, right_idx):
            if ranges[i] <= self.safe_distance:
                count += 1
        return count > self.instance_threshold

    def complete_transition_to_a(self):
        """Completes the transition to Node A after the delay."""
        self.get_logger().info("Transition to Node A complete.")
        self.use_node_b = False
        self.transitioning_to_a = False
        if self.transition_timer:
            self.transition_timer.cancel()
            self.transition_timer = None

    def node_a_callback(self, msg):
        """Stores the latest message from Node A."""
        self.last_msg_a = msg

    def node_b_callback(self, msg):
        """Stores the latest message from Node B."""
        self.last_msg_b = msg

    def publish_drive_command(self):
        """Publishes the drive command from the selected node."""
        if self.use_node_b and self.last_msg_b:
            self.drive_publisher.publish(self.last_msg_b)
        elif not self.use_node_b and self.last_msg_a:
            self.drive_publisher.publish(self.last_msg_a)


def main(args=None):
    rclpy.init(args=args)
    mux_node = MuxNode()
    rclpy.spin(mux_node)
    mux_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

