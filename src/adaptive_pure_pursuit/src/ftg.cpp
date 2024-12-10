#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>

using namespace std;

class GapFollower : public rclcpp::Node {
public:
    GapFollower()
        : Node("gap_follower"),
          blur_size_(25),
          safe_distance_(1.5),
          danger_distance_(0.8),
          jump_threshold_(1.0),
          max_velocity_(3.0),
          min_velocity_(1.0),
          max_gap_size_(400),
          min_gap_size_(100),
          smoothing_window_(15),
          alpha_(0.3),
          ema_steering_angle_(0.0) {
        
        // Publishers
        //steering_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/follow_the_gap", 10);  //"/drive"
        //throttle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);

        // Subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,

            std::bind(&GapFollower::lidarCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "GapFollower Node Initialized");
    }

private:
    //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_publisher_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    // Constants
    int blur_size_;
    double safe_distance_;
    double danger_distance_;
    double jump_threshold_;
    double max_velocity_;
    double min_velocity_;
    int max_gap_size_;
    int min_gap_size_;
    int smoothing_window_;
    double alpha_;
    double ema_steering_angle_;

    // Helper functions
    std::vector<float> filterLidarReadings(const std::vector<float> &ranges) {
        std::vector<float> smoothed_ranges(ranges.size());
        int window = smoothing_window_;
        for (size_t i = 0; i < ranges.size(); i += window) {
            float sum = 0.0f;
            size_t end = std::min(ranges.size(), i + window);
            for (size_t j = i; j < end; ++j) {
                sum += ranges[j];
            }
            float avg = sum / (end - i);
            std::fill(smoothed_ranges.begin() + i, smoothed_ranges.begin() + end, avg);
        }
        return smoothed_ranges;
    }

    void maskDangerZones(size_t start_idx, size_t end_idx, std::vector<float> &ranges) {
        for (size_t i = start_idx; i < end_idx; ++i) {
            if (ranges[i] <= danger_distance_) {
                size_t start = (i > blur_size_) ? i - blur_size_ : 0;
                size_t end = std::min(i + blur_size_, ranges.size());
                std::fill(ranges.begin() + start, ranges.begin() + end, 0.0f);
                i += blur_size_;
            }
        }
    }

    int selectBestHeading(size_t start_idx, size_t end_idx, const std::vector<float> &ranges) {
        std::priority_queue<std::pair<float, std::pair<size_t, size_t>>> safe_ranges;
        for (size_t i = start_idx; i < end_idx; ++i) {
            if (ranges[i] >= safe_distance_) {
                size_t safe_start = i;
                while (i < end_idx && ranges[i] >= safe_distance_ && (i - safe_start) <= max_gap_size_ &&
                       std::abs(ranges[i] - ranges[i > 0 ? i - 1 : 0]) < jump_threshold_) {
                    ++i;
                }
                size_t safe_end = i - 1;
                if (safe_end > safe_start) {
                    float max_range = *std::max_element(ranges.begin() + safe_start, ranges.begin() + safe_end);
                    safe_ranges.emplace(-max_range, std::make_pair(safe_start, safe_end));
                }
            }
        }
        if (safe_ranges.empty()) {
            return std::distance(ranges.begin(), std::max_element(ranges.begin(), ranges.end()));
        }

        while (!safe_ranges.empty()) {
            auto [_, range] = safe_ranges.top();
            size_t safe_start = range.first;
            size_t safe_end = range.second;
            safe_ranges.pop();
            if (safe_end - safe_start > min_gap_size_) {
                return (safe_start + safe_end) / 2;
            }
        }
        return std::distance(ranges.begin(), std::max_element(ranges.begin(), ranges.end()));
    }

    // Lidar callback
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto ranges = msg->ranges;
        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;

        auto processed_ranges = filterLidarReadings(ranges);
        size_t closest_idx = std::min_element(processed_ranges.begin(), processed_ranges.end()) - processed_ranges.begin();

        maskDangerZones(0, processed_ranges.size(), processed_ranges);
        int farthest_idx = selectBestHeading(0, processed_ranges.size(), processed_ranges);

        double steering_angle = angle_min + farthest_idx * angle_increment;
        ema_steering_angle_ = alpha_ * steering_angle + (1.0 - alpha_) * ema_steering_angle_;

        // Adjust velocity based on steering angle
        double velocity = max_velocity_ - (std::min(std::abs(ema_steering_angle_), 0.5) / 0.5) * (max_velocity_ - min_velocity_);

	ackermann_msgs::msg::AckermannDriveStamped drive_msg;
	drive_msg.header.stamp = this->now();
	drive_msg.header.frame_id = "base_link";

	drive_msg.drive.steering_angle = ema_steering_angle_;
    	drive_msg.drive.speed = velocity;

    	drive_publisher_->publish(drive_msg);
        RCLCPP_INFO(this->get_logger(), "Steering Angle: %.2f degrees, Velocity: %.2f", 
                    ema_steering_angle_ * 180.0 / M_PI, velocity);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GapFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
