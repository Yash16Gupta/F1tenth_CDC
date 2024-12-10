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
        print("oood")
        # Publishers and Subscribers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        
        # Subscriptions to Node A and Node B topics
        self.node_a_subscriber = self.create_subscription(
            AckermannDriveStamped, self.node_a_topic, self.node_a_callback, 10)
        self.node_b_subscriber = self.create_subscription(
            AckermannDriveStamped, self.node_b_topic, self.node_b_callback, 10)

        # LiDAR Subscription
        self.lidar_subscriber = self.create_subscription(
            LaserScan, self.lidar_topic, self.lidar_callback, 10)

        # State
        self.use_node_b = False  # Start with Node A by default
        self.last_msg_a = None
        self.last_msg_b = None

        # Constants
        self.safe_distance = 1.0  # Threshold to detect obstacles in m
        self.detection_angle = 45  # Field of view for obstacle detection in degree
        self.instance_threshold = 1 #n.points
    def lidar_callback(self, scan_msg):
        """Processes LiDAR data to check for obstacles."""
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min

        # Check for obstacles within the detection angle and safe distance
        obstacle_detected = self.detect_obstacle(ranges, angle_increment, angle_min)

        if obstacle_detected:
            self.get_logger().info("Obstacle detected! Switching to Node B.")
            self.use_node_b = True
        else:
            self.get_logger().info("No obstacle detected. Using Node A.")
            self.use_node_b = False

        # Publish the latest message from the selected node
        self.publish_drive_command()

    def detect_obstacle(self, ranges, angle_increment, angle_min):
        """Detect obstacles within a specified field of view and safe distance."""
        center_idx = len(ranges) // 2
        fov_indices = int(self.detection_angle / 2 / (angle_increment * 180 / np.pi))

        left_idx = max(0, center_idx - fov_indices)
        right_idx = min(len(ranges), center_idx + fov_indices)
        count=0
        # Check for obstacles within the safe distance
        for i in range(left_idx, right_idx):
            if ranges[i] <= self.safe_distance:
                count+=1
        if count>self.instance_threshold:
            return True
        return False

    def node_a_callback(self, msg):
        """Stores the latest message from Node A."""
        self.last_msg_a = msg

    def node_b_callback(self, msg):
        """Stores the latest message from Node B."""
        self.last_msg_b = msg

    def publish_drive_command(self):
        """Publishes the drive command from the selected node."""
        if self.use_node_b and self.last_msg_b:
            #self.get_logger().info("Publishing from Node B")
            print("A")
            self.drive_publisher.publish(self.last_msg_b)
        elif not self.use_node_b and self.last_msg_a:
            #self.get_logger().info("Publishing from Node A")
            print("B")
            self.drive_publisher.publish(self.last_msg_a)


def main(args=None):
    rclpy.init(args=args)
    mux_node = MuxNode()
    rclpy.spin(mux_node)
    mux_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
