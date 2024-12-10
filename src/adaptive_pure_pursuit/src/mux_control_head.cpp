#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>

struct Obstacle {
    double distance;
    double angle;
    bool is_wall;
    double confidence_score;
};

struct Point {
    double x;
    double y;
};

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node"),
                              tf_buffer_(get_clock()),
                              tf_listener_(tf_buffer_) {
        declare_parameter<std::string>("raceline_csv", "/home/nvidia/Downloads/sahil_mc_raceline.csv");
        auto filename = get_parameter("raceline_csv").as_string();
        loadRacelineFromCSV(filename);

        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10), std::bind(&ObstacleAvoidanceNode::lidarCallback, this, std::placeholders::_1));
        pure_pursuit_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/pure_pursuit", rclcpp::QoS(10), std::bind(&ObstacleAvoidanceNode::purePursuitCallback, this, std::placeholders::_1));
        ftg_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/follow_the_gap", rclcpp::QoS(10), std::bind(&ObstacleAvoidanceNode::ftgCallback, this, std::placeholders::_1));
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", rclcpp::QoS(10));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pure_pursuit_subscriber_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ftg_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ackermann_msgs::msg::AckermannDriveStamped pure_pursuit_output_;
    ackermann_msgs::msg::AckermannDriveStamped ftg_output_;
    std::vector<Point> raceline_;
    std::vector<Obstacle> obstacles_;
    std::vector<std::vector<Obstacle>> clusters_;

    const double critical_distance_ = 2.0;
    const double confidence_threshold_ = 0.7;
    const double base_cone_angle_ = 45.0;
    const double raceline_corridor_ = 0.5;
    const double cluster_distance_threshold_ = 0.5;
    const double wall_length_threshold_ = 1.0;
    const double safety_speed_ = 2.3;

    void loadRacelineFromCSV(const std::string &filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open raceline CSV: %s", filename.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            Point point;

            std::getline(ss, value, ',');
            point.x = std::stod(value);
            std::getline(ss, value, ',');
            point.y = std::stod(value);

            raceline_.push_back(point);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded raceline with %zu points.", raceline_.size());
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        obstacles_.clear();

        geometry_msgs::msg::PoseStamped car_pose;
        if (!getCarPose(car_pose)) {
            return;
        }

        Point closest_point = getClosestRacelinePoint(car_pose.pose.position.x, car_pose.pose.position.y);
        double target_angle = std::atan2(closest_point.y - car_pose.pose.position.y,
                                         closest_point.x - car_pose.pose.position.x);
        double dynamic_cone_angle = getDynamicConeAngle(car_pose.pose.position.x, car_pose.pose.position.y);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double distance = msg->ranges[i];

            if (distance < msg->range_min || distance > msg->range_max) continue;
            if (std::abs(angle - target_angle) > dynamic_cone_angle) continue;

            obstacles_.emplace_back(Obstacle{distance, angle, false, 0.0});
        }

        clusterObstacles();
        classifyObstacles();
        decideDrivingStrategy();
    }

    bool getCarPose(geometry_msgs::msg::PoseStamped &car_pose) {
        try {
            auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            car_pose.pose.position.x = transform.transform.translation.x;
            car_pose.pose.position.y = transform.transform.translation.y;
            car_pose.pose.orientation = transform.transform.rotation;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return false;
        }
    }

    double getDynamicConeAngle(double car_x, double car_y) {
        Point closest_point = getClosestRacelinePoint(car_x, car_y);
        double distance_to_raceline = std::hypot(closest_point.x - car_x, closest_point.y - car_y);
        double adjusted_angle = base_cone_angle_ * (1.0 - std::min(distance_to_raceline / raceline_corridor_, 1.0));
        return std::max(adjusted_angle * M_PI / 180.0, 0.05);
    }

    Point getClosestRacelinePoint(double car_x, double car_y) {
        Point closest_point;
        double min_distance = std::numeric_limits<double>::max();
        for (const auto &point : raceline_) {
            double distance = std::hypot(point.x - car_x, point.y - car_y);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
            }
        }
        return closest_point;
    }

    void clusterObstacles() {
        clusters_.clear();
        std::sort(obstacles_.begin(), obstacles_.end(), [](const Obstacle &a, const Obstacle &b) {
            return a.distance < b.distance;
        });

        for (const auto &obs : obstacles_) {
            bool added = false;
            for (auto &cluster : clusters_) {
                if (std::abs(cluster.back().distance - obs.distance) < cluster_distance_threshold_) {
                    cluster.push_back(obs);
                    added = true;
                    break;
                }
            }
            if (!added) {
                clusters_.emplace_back(std::vector<Obstacle>{obs});
            }
        }
    }

    void classifyObstacles() {
        for (auto &cluster : clusters_) {
            double cluster_length = cluster.back().distance - cluster.front().distance;
            for (auto &obs : cluster) {
                obs.is_wall = cluster_length > wall_length_threshold_;
                obs.confidence_score = obs.is_wall ? 0.5 : 1.0;
            }
        }
    }

    void purePursuitCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        pure_pursuit_output_ = *msg;
    }

    void ftgCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        ftg_output_ = *msg;
    }

    void decideDrivingStrategy() {
        geometry_msgs::msg::PoseStamped car_pose;
        if (!getCarPose(car_pose)) {
            return;
        }

        bool obstacles_near_raceline = false;
        for (const auto &obs : obstacles_) {
            if (!obs.is_wall && isObstacleInRacelineCorridor(obs, car_pose)) {
                obstacles_near_raceline = true;
                break;
            }
        }

        double confidence_pp = calculateConfidence(pure_pursuit_output_);
        double confidence_ftg = calculateConfidence(ftg_output_);

        if (!obstacles_near_raceline) {
            confidence_pp += 0.5;
        } else {
            confidence_ftg += 0.5;
        }

        ackermann_msgs::msg::AckermannDriveStamped final_output;
        if (confidence_ftg > confidence_pp) {
            final_output = ftg_output_;
            std::cout<<"ftg"<<std::endl;
        } else {
            final_output = pure_pursuit_output_;
            std::cout<<"PPPP"<<std::endl;
        }

        if (confidence_pp < 0.2 && confidence_ftg < 0.2) {
            final_output.drive.speed = safety_speed_;
            final_output.drive.steering_angle = 0.0;
            std::cout<<"DOA"<<std::endl;

        }
        RCLCPP_INFO(this->get_logger(), "Final Output: Speed = %.2f, Steering Angle = %.2f",
                    final_output.drive.speed, final_output.drive.steering_angle);
        drive_publisher_->publish(final_output);
    }

    bool isObstacleInRacelineCorridor(const Obstacle &obs, const geometry_msgs::msg::PoseStamped &car_pose) {
        double obstacle_x = car_pose.pose.position.x + obs.distance * std::cos(obs.angle);
        double obstacle_y = car_pose.pose.position.y + obs.distance * std::sin(obs.angle);

        Point closest_point = getClosestRacelinePoint(car_pose.pose.position.x, car_pose.pose.position.y);
        double distance_to_raceline = std::hypot(obstacle_x - closest_point.x, obstacle_y - closest_point.y);
        return distance_to_raceline <= raceline_corridor_;
    }

    double calculateConfidence(const ackermann_msgs::msg::AckermannDriveStamped &command) {
        double confidence = 1.0;
        for (const auto &obs : obstacles_) {
            if (obs.distance < critical_distance_) {
                confidence -= 0.5 * (critical_distance_ - obs.distance) / critical_distance_;
            }
        }
        return std::max(confidence, 0.0);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
