#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <deque>
#include <cmath>
#include <optional>
#include <algorithm>
#include <numeric>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class AdaptivePurePursuit : public rclcpp::Node
{
public:
    AdaptivePurePursuit()
        : Node("adaptive_pure_pursuit"),

          heading_angle_(0.5),
          previous_deviation_(0.0),
          total_area_(0.0),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {

        max_speed_ = this->declare_parameter("max_speed", 20.0); //11.0
        min_speed_ = this->declare_parameter("min_speed", 16.0); // 7.0
        max_lookahead_ = this->declare_parameter("max_lookahead", 1.3); //1.6
        min_lookahead_ = this->declare_parameter("min_lookahead", 1.0); //1.3
        wheelbase_ = this->declare_parameter("wheelbase", 0.33);
        beta_ = this->declare_parameter("beta", 0.5);
        heading_scale_ = this->declare_parameter("heading_scale", 1.1);
        area_threshold_ = this->declare_parameter("area_threshold", 1.0);
        speed_factor_ = this->declare_parameter("speed_factor", 0.3);
        velocity_superintendence_1_ = this->declare_parameter("velocity_superintendence_1", 2.1);
        velocity_superintendence_2_ = this->declare_parameter("velocity_superintendence_2", 0.8);
        window_size_ = this->declare_parameter("window_size", 5);
        control_velocity_ = this->declare_parameter("control_velocity", 0.1);
        
	this->declare_parameter("a_", 1);

//	rclcpp::Parameter test  = this->get_parameter("a_");
//        int a_ = test.as_int();


        // std::string raceline_csv_path = this->declare_parameter("raceline_csv_path", "");
	

        lookahead_distance_ = min_lookahead_;
        current_quaternion_ = {0.0, 0.0, 0.0, 1.0};
        initialized_ = false;


        pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AdaptivePurePursuit::pos_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);  // "/drive"

        goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/goal", 10);

        cp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/cp", 10);

        race_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/raceline", 10);

        load_raceline_csv("/home/nvidia/ros2_ws/src/adaptive_pure_pursuit/final_test.csv");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&AdaptivePurePursuit::publish_control_commands, this));

        
        RCLCPP_INFO(this->get_logger(), "Adaptive Pure Pursuit Node Initialized.");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thr_pub_, str_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_, cp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr race_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Eigen::Vector2d> path_;
    std::optional<Eigen::Vector2d> previous_position_;
    std::optional<Eigen::Vector2d> current_position_;
    std::deque<double> area_window_;
    bool initialized_;

    std::array<double, 4> current_quaternion_;

    double max_speed_, min_speed_, max_lookahead_, min_lookahead_, wheelbase_;
    double lookahead_distance_, beta_, previous_deviation_, total_area_, control_velocity_, heading_angle_, a_;
    double heading_scale_, area_threshold_, speed_factor_;
    double velocity_superintendence_1_, velocity_superintendence_2_;
    size_t window_size_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void load_raceline_csv(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
	    // RCLCPP_INFO(this->get_logger(), "got built");
            return;
        }

        std::string line;
        std::vector<Eigen::Vector2d> temp_path;

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string x_str, y_str;

            std::getline(ss, x_str, ',');
            std::getline(ss, y_str);

            double x = std::stod(x_str);
            double y = std::stod(y_str);

            temp_path.emplace_back(x, y);
        }

        if(a_ == 1.0){
            std::reverse(temp_path.begin(), temp_path.end());
            path_ = temp_path;
            RCLCPP_INFO(this->get_logger(), "1.0");

        } else if(a_ == 0.0){
            path_ = temp_path;
            RCLCPP_INFO(this->get_logger(), "0.0");

        }
        // for (auto &point : temp_path)
        // {
        //     point[0] += 0;
        //     point[1] += 0;
        // }

        // Eigen::Matrix2d rotation_matrix;
        // rotation_matrix << 1, 0.0,
        //     0.0, 1.0;

        // for (auto &point : temp_path)
        // {
        //     Eigen::Vector2d transformed_point = rotation_matrix * point;
        //     path_.emplace_back(transformed_point);
        // }
    }

    void pos_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0), std::chrono::seconds(10));
	

	current_position_ = Eigen::Vector2d(transform.transform.translation.x, transform.transform.translation.y);
        
        current_quaternion_ = {
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w};

        if (!initialized_)
        {
            previous_position_ = current_position_.value();
            initialized_ = true;
        }

        double yaw = quaternion_to_yaw(
            current_quaternion_[0],
            current_quaternion_[1],
            current_quaternion_[2],
            current_quaternion_[3]);

        double current_speed = msg->twist.twist.linear.x;

        update_lookahead_distance(current_speed);
        auto [closest_point, goal_point] = find_lookahead_point();
        if (goal_point.has_value())
        {
            double alpha = calculate_alpha(goal_point.value(), yaw);
            heading_angle_ = calculate_heading_angle(alpha);
            double area = calculate_deviation(current_position_.value(), closest_point);
            double max_velocity_pp = calculate_max_velocity_pure_pursuit(calculate_curvature(alpha));
            double min_deviation_pp = calculate_min_deviation_pure_pursuit(area);
            control_velocity_ = convex_combination(max_velocity_pp, min_deviation_pp, current_speed, area);
            publish_markers(closest_point, goal_point.value());
            publish_raceline_visualization();
            publish_control_commands();
        }
    }

    double quaternion_to_yaw(double x, double y, double z, double w)
    {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void update_lookahead_distance(double speed)
    {
        double normalized_speed = (speed - min_speed_) / (max_speed_ - min_speed_);
        double sigmoid_value = 1.0 / (1.0 + std::exp(-(normalized_speed * 10 - 5)));

        if (speed < min_speed_)
        {
            lookahead_distance_ = min_lookahead_;
        }
        else
        {
            double scaled_lookahead = min_lookahead_ + sigmoid_value * (max_lookahead_ - min_lookahead_);
            lookahead_distance_ = std::min(max_lookahead_, scaled_lookahead);
        }
    }

    std::pair<Eigen::Vector2d, std::optional<Eigen::Vector2d>> find_lookahead_point()
    {
        Eigen::Vector2d closest_point;
        std::optional<Eigen::Vector2d> goal_point;
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_point_index = 0;

        for (size_t i = 0; i < path_.size(); ++i)
        {
            double dist = (path_[i] - current_position_.value()).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_point = path_[i];
                closest_point_index = i;
            }
        }

        for (size_t i = 0; i < path_.size(); ++i)
        {
            double dist = (path_[i] - current_position_.value()).norm();
            if (dist > lookahead_distance_ &&
                i > closest_point_index + 1 && i < closest_point_index + 10)
            {
                goal_point = path_[i];
                break;
            }
        }

        return {closest_point, goal_point};
    }

    double calculate_alpha(const Eigen::Vector2d &goal_point, double yaw)
    {
        Eigen::Vector2d delta = goal_point - current_position_.value();
        double local_x = delta.x() * std::cos(-yaw) - delta.y() * std::sin(-yaw);
        double local_y = delta.x() * std::sin(-yaw) + delta.y() * std::cos(-yaw);
        return std::atan2(local_y, local_x);
    }

    double calculate_heading_angle(double alpha)
    {
        return std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance_);
    }

    double calculate_curvature(double alpha)
    {
        return 2.0 * std::sin(alpha) / lookahead_distance_;
    }

    double calculate_deviation(const Eigen::Vector2d &position, const Eigen::Vector2d &closest_point)
    {
        double deviation = (closest_point - position).norm();

        if (previous_position_.has_value())
        {
            double distance_traveled = (position - previous_position_.value()).norm();

            double area_increment = (deviation + previous_deviation_) / 2.0 * distance_traveled;

            area_window_.push_back(area_increment);

            if (area_window_.size() > window_size_)
            {
                area_window_.pop_front();
            }
            total_area_ = std::accumulate(area_window_.begin(), area_window_.end(), 0.0);
        }

        previous_position_ = position;
        previous_deviation_ = deviation;

        return total_area_;
    }

    double calculate_max_velocity_pure_pursuit(double curvature)
    {
        double max_velocity = (curvature != 0.0) ? std::sqrt(1.0 / std::abs(curvature)) : max_speed_;
        return std::min(max_speed_, max_velocity);
    }

    double calculate_min_deviation_pure_pursuit(double area)
    {
        double min_deviation_velocity;
        if (area > 0.0)
        {
            min_deviation_velocity = max_speed_ / (1.0 + area);
        }
        else
        {
            min_deviation_velocity = max_speed_;
        }
        return min_deviation_velocity;
    }

    double adjust_beta(double current_speed, double area)
    {
        if (area < area_threshold_)
        {
            return std::min(1.0, beta_ + 0.25);
        }
        else if (current_speed < max_speed_ * speed_factor_)
        {
            return std::max(0.0, beta_ - 0.25);
        }
        return beta_;
    }

    double convex_combination(double max_velocity_pp, double min_deviation_pp, double current_speed, double area)
    {
        beta_ = adjust_beta(current_speed, area);

        double control_velocity = beta_ * max_velocity_pp + (1.0 - beta_) * min_deviation_pp;
        //RCLCPP_INFO(this->get_logger(), "working");

        // double curvature = calculate_curvature(heading_angle_);
        // double curv_diff = std::abs(curvature);

        // control_velocity /= std::exp(velocity_superintendence_1_ * std::pow(std::abs(curv_diff), velocity_superintendence_2_));

        return control_velocity;
    }

    void publish_markers(const Eigen::Vector2d &closest_point, const Eigen::Vector2d &goal_point)
    {
        auto create_marker = [&](const Eigen::Vector2d &point, float r, float g, float b)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.x();
            marker.pose.position.y = point.y();
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 1.0;
            return marker;
        };

        cp_pub_->publish(create_marker(closest_point, 0.0, 0.0, 1.0));
        goal_pub_->publish(create_marker(goal_point, 1.0, 0.0, 0.0));
    }

    void publish_raceline_visualization()
    {
        visualization_msgs::msg::MarkerArray raceline_markers;
        int id = 0;
        for (const auto &point : path_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.x();
            marker.pose.position.y = point.y();
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.09;
            marker.scale.y = 0.09;
            marker.scale.z = 0.09;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.id = id++;
            raceline_markers.markers.push_back(marker);
        }
        race_pub_->publish(raceline_markers);
    }

    void publish_control_commands()
    {
        auto drive_msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
        drive_msg->header.frame_id = "map";
        drive_msg->header.stamp = this->get_clock()->now();

        drive_msg->drive.speed = control_velocity_;
        drive_msg->drive.steering_angle = std::clamp( heading_angle_* heading_scale_, -0.24, 0.24); 
	   // RCLCPP_INFO(this->get_logger(),
                //    "steer: %.2f, velocity: %.2f",
                  //  heading_angle_ * heading_scale_ * (180.0 / M_PI),
                    //control_velocity_);

        drive_pub_->publish(*drive_msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptivePurePursuit>());
    rclcpp::shutdown();
    return 0;
}
