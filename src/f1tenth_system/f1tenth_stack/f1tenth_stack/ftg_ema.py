import rclpy
from rclpy.node import Node
import math
from queue import PriorityQueue

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class GapFollower(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        # Topics & Publishers, Subscribers
        lidarscan_topic = '/scan'
        drive_topic = '/ftg_cmd'  
        print("wor")
        # Initialize publisher for AckermannDriveStamped
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)

        # Subscribe to Lidar topic
        self.scan_subscriber = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, 10
        )

        # Constants
        self.blur_size = 35
        self.safe_distance = 1.7
        self.danger_distance = 1.0
        self.jump_threshold = 1.0
        self.max_velocity = 3.5
        self.min_velocity = 1.0
        self.max_gap_size = 400
        self.min_gap_size = 100
        self.smoothing_window = 15
        self.ema_alpha_steering = 0.1  # Smoothing factor for EMA of steering
        self.ema_alpha_velocity = 0.1  # Smoothing factor for EMA of velocity

        # State
        self.steering_ema = None
        self.velocity_ema = None

    def filter_lidar_readings(self, ranges):
        smoothed_ranges = []
        window = self.smoothing_window
        for i in range(0, len(ranges), window):
            current_mean = round(sum(ranges[i:i + window]) / window, 5)
            smoothed_ranges.extend([current_mean] * window)
        return np.array(smoothed_ranges)

    def mask_danger_zones(self, start_idx, end_idx, ranges):
        idx = start_idx
        while idx < end_idx:
            if ranges[idx] <= self.danger_distance:
                ranges[max(0, idx - self.blur_size): idx + self.blur_size] = 0
                idx += self.blur_size
            else:
                idx += 1
        return ranges

    def select_best_heading(self, start_idx, end_idx, closest_idx, ranges):
        idx = start_idx
        safe_ranges = PriorityQueue()
        while idx < end_idx:
            if ranges[idx] >= self.safe_distance:
                safe_start = idx
                idx += 1
                while (idx < end_idx and ranges[idx] >= self.safe_distance
                       and idx - safe_start <= self.max_gap_size
                       and abs(ranges[idx] - ranges[max(0, idx - 1)]) < self.jump_threshold):
                    idx += 1
                safe_end = max(0, idx - 1)
                if safe_end != safe_start:
                    safe_ranges.put(
                        (-(np.max(ranges[safe_start:safe_end])), (safe_start, safe_end)))
            else:
                idx += 1
        if safe_ranges.empty():
            self.get_logger().info('No safe ranges found')
            return np.argmax(ranges)
        else:
            while not safe_ranges.empty():
                safe_start, safe_end = safe_ranges.get()[1]
                target_idx = (safe_start + safe_end) // 2
                if 179 <= target_idx <= 900 and safe_end - safe_start > self.min_gap_size:
                    self.get_logger().info(f'left: {safe_start}, right: {safe_end}')
                    return target_idx
            return target_idx

    def lidar_callback(self, scan_msg):
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min
        ranges = scan_msg.ranges

        processed_ranges = self.filter_lidar_readings(ranges)
        closest_point_idx = np.argmin(processed_ranges)

        processed_ranges = self.mask_danger_zones(
            0, len(processed_ranges), processed_ranges)

        farthest_point_idx = self.select_best_heading(
            0, len(processed_ranges), closest_point_idx, processed_ranges)
        steering_angle = angle_min + farthest_point_idx * angle_increment

        # Apply Exponential Moving Average to the steering angle
        if self.steering_ema is None:  # Initialize EMA with the first angle
            self.steering_ema = steering_angle
        else:
            self.steering_ema = self.ema_alpha_steering * steering_angle + \
                (1 - self.ema_alpha_steering) * self.steering_ema

        # Calculate velocity based on steering angle
        steering_angle_limit = 0.5  # Limit for maximum steering angle in radians
        raw_velocity = self.max_velocity - (
            min(abs(self.steering_ema), steering_angle_limit) / steering_angle_limit
        ) * (self.max_velocity - self.min_velocity)

        # Apply Exponential Moving Average to the velocity
        if self.velocity_ema is None:  # Initialize EMA with the first velocity
            self.velocity_ema = raw_velocity
        else:
            self.velocity_ema = self.ema_alpha_velocity * raw_velocity + \
                (1 - self.ema_alpha_velocity) * self.velocity_ema

        # Publish AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.velocity_ema
        drive_msg.drive.steering_angle = self.steering_ema
        self.drive_publisher.publish(drive_msg)

        # Logging information
        self.get_logger().info(f'Farthest point index: {farthest_point_idx}')
        self.get_logger().info(
            f'Range at farthest point: {processed_ranges[farthest_point_idx]}')
        self.get_logger().info(
            f'Smoothed Steering angle: {math.degrees(self.steering_ema):.2f}')
        self.get_logger().info(f'Smoothed Velocity: {self.velocity_ema:.2f}')


def main(args=None):
    rclpy.init(args=args)
    print("GapFollower Node Initialized")
    follow_the_gap = GapFollower()
    rclpy.spin(follow_the_gap)

    follow_the_gap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
