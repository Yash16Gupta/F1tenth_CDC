import numpy as np
import rclpy
import rclpy.logging
import tf_transformations
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pow, sin, exp, cos
from ackermann_msgs.msg import AckermannDriveStamped
import pandas as pd
import time


class AdaptivePurePursuit(Node):
    def __init__(self):
        super().__init__('adaptive_pure_pursuit')

        print("mewmew")
        self.load_raceline_csv(
            '/home/nvidia/Downloads/sahil_mc_raceline.csv')
        self.pos_sub = self.create_subscription(
            Odometry, '/odom', self.pos_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal', 10)
        self.cp_pub = self.create_publisher(Marker, '/cp', 10)
        self.race_pub = self.create_publisher(MarkerArray, '/raceline', 10)

        timer = 0.0001
        self.timer = self.create_timer(timer, self.publish_control_commands)
        self.max_speed = 2
        self.min_speed = 1
        self.max_lookahead = 2.5
        self.min_lookahead = 1.5
        self.wheelbase = 0.33
        self.current_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.lookahead_distance = self.min_lookahead
        self.beta = 0.5
        self.previous_goal = [0, 0]
        self.position = None
        self.orientation = None
        self.laser_scan_data = []  # For storing laser scan data
        self.control_velocity=0.0
        self.heading_angle=0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def load_raceline_csv(self, filename):
        self.path = pd.read_csv(filename)
        self.path = np.array([self.path]).reshape(-1, 2)
        self.path = self.path[::3]
        self.path = np.vstack((self.path, self.path[:10]))

    def laser_callback(self, msg):
        """
        Callback for laser scan data.
        Stores the ranges array from the LaserScan message.
        """
        self.laser_scan_data = msg.ranges

    def pos_callback(self, msg):
        try:
            print("pos_get")
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=10.0))

            self.position = transform.transform.translation
            self.orientation = transform.transform.rotation

            self.current_quaternion = [
                self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
            self.yaw = self.quaternion_to_yaw(self.current_quaternion)

            current_speed = msg.twist.twist.linear.x
            self.update_lookahead(current_speed)

            closest_point, goal_point = self.get_lookahead_point(
                self.position, self.laser_scan_data)

            if goal_point is not None:
                self.previous_goal = goal_point
                alpha = self.calculate_alpha(
                    self.position, goal_point, self.yaw)
                self.heading_angle = self.calculate_heading_angle(alpha)
                self.control_velocity = self.max_speed
                self.publish_control_commands()
            else:
                goal_point = self.previous_goal

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"Transform exception failed: {e}")

    def quaternion_to_yaw(self, quaternion):
        qx, qy, qz, qw = quaternion
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return atan2(siny_cosp, cosy_cosp)

    def update_lookahead(self, speed):
        normalized_speed = (speed - self.min_speed) / \
            (self.max_speed - self.min_speed)
        sigmoid_value = 1 / (1 + exp(-normalized_speed * 10 - 5))
        self.lookahead_distance = self.min_lookahead + sigmoid_value * \
            (self.max_lookahead - self.min_lookahead)

    def get_lookahead_point(self, position, laser_scan_data):
        min_dist = float('inf')
        closest_point = None
        goal_point = None
        for point in self.path:
            dist = sqrt(pow(point[0] - position.x, 2) +
                        pow(point[1] - position.y, 2))
            if dist < min_dist:
                min_dist = dist
                closest_point = point

        for point in self.path:
            dist = sqrt(pow(point[0] - position.x, 2) +
                        pow(point[1] - position.y, 2))
            if dist > self.lookahead_distance and self.has_line_of_sight(position, point, laser_scan_data):
                goal_point = point
                break
        return closest_point, goal_point

    def has_line_of_sight(self, position, goal_point, laser_scan_data):
        """
        Check if there is a clear line of sight between the car's position and the goal point.
        """
        angle_to_goal = atan2(goal_point[1] - position.y,
                              goal_point[0] - position.x)
        distance_to_goal = sqrt(pow(goal_point[0] - position.x, 2) +
                                pow(goal_point[1] - position.y, 2))
        index = int((angle_to_goal + np.pi) / (2 * np.pi) *
                    len(laser_scan_data)) % len(laser_scan_data)
        return laser_scan_data[index] > distance_to_goal

    def calculate_alpha(self, position, goal_point, yaw):
        dy = goal_point[1] - position.y
        dx = goal_point[0] - position.x
        local_x = dx * cos(-yaw) - dy * sin(-yaw)
        local_y = dx * sin(-yaw) + dy * cos(-yaw)
        return atan2(local_y, local_x)

    def calculate_heading_angle(self, alpha):
        return atan2(2 * self.wheelbase * sin(alpha), self.lookahead_distance)

    def publish_control_commands(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = 'map'
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        #drive_msg.drive.speed = self.control_velocity
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = self.heading_angle
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    adaptive_pure_pursuit = AdaptivePurePursuit()
    try:
        rclpy.spin(adaptive_pure_pursuit)
    except KeyboardInterrupt:
        pass
    adaptive_pure_pursuit.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
