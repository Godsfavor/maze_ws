#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "real_maze_robots/room_manager.hpp"

using namespace std::chrono_literals;

class RealRobotController : public rclcpp::Node {
public:
    RealRobotController() : Node("real_robot_controller") {
        // Parameters
        this->declare_parameter<std::string>("robot_name", "robot1");
        this->declare_parameter<std::string>("initial_room", "room_1");
        this->declare_parameter<double>("safe_distance", 0.30);   // More conservative for real robots
        this->declare_parameter<double>("slow_distance", 0.50);

        robot_name_ = this->get_parameter("robot_name").as_string();
        current_room_ = this->get_parameter("initial_room").as_string();
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        slow_distance_ = this->get_parameter("slow_distance").as_double();

        RCLCPP_INFO(this->get_logger(), "🤖 Starting %s in %s", 
                    robot_name_.c_str(), current_room_.c_str());

        // Publishers - NO namespace prefix (TurtleBot topics are already global)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Subscribers - TurtleBot 3 publishes to these topics
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RealRobotController::odomCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),  // CRITICAL for TurtleBot 3!
            std::bind(&RealRobotController::scanCallback, this, std::placeholders::_1));

        // Command topics (shared across all robots via ROS_DOMAIN_ID)
        left_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/left", 10,
            std::bind(&RealRobotController::leftCallback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/right", 10,
            std::bind(&RealRobotController::rightCallback, this, std::placeholders::_1));

        // Control loop - slower for real robots
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RealRobotController::controlLoop, this));

        // Initialize state
        state_ = State::IDLE;
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        target_x_ = 0.0;
        target_y_ = 0.0;
        min_obstacle_distance_ = 10.0;
        is_path_blocked_ = false;

        RCLCPP_INFO(this->get_logger(), "✅ Ready! Waiting for /left or /right signal");
        RCLCPP_INFO(this->get_logger(), "📡 ROS_DOMAIN_ID: %s", 
                    std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "NOT SET");
    }

private:
    enum class State {
        IDLE,
        ROTATING,
        MOVING_FORWARD,
        OBSTACLE_WAIT,
        ARRIVED
    };

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        // Debug: Print position every 2 seconds
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Position: (%.2f, %.2f) Yaw: %.2f", current_x_, current_y_, current_yaw_);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check narrow front cone (±30 degrees for real robot)
        int total_rays = msg->ranges.size();
        int cone_width = total_rays / 6;  // 60 degrees total
        
        int front_start = 0;
        int front_end = cone_width / 2;
        int back_start = total_rays - (cone_width / 2);
        
        min_obstacle_distance_ = 10.0;

        // Check front cone
        for (int i = front_start; i <= front_end; i++) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range > msg->range_min && range < msg->range_max) {
                min_obstacle_distance_ = std::min(min_obstacle_distance_, (double)range);
            }
        }
        
        // Check back cone
        for (int i = back_start; i < total_rays; i++) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range > msg->range_min && range < msg->range_max) {
                min_obstacle_distance_ = std::min(min_obstacle_distance_, (double)range);
            }
        }

        is_path_blocked_ = (min_obstacle_distance_ < safe_distance_);
    }

    void leftCallback(const example_interfaces::msg::Empty::SharedPtr /*msg*/) {
        if (state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "⬅️  LEFT signal received");
            startMovement(false);
        }
    }

    void rightCallback(const example_interfaces::msg::Empty::SharedPtr /*msg*/) {
        if (state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "➡️  RIGHT signal received");
            startMovement(true);
        }
    }

    void startMovement(bool clockwise) {
        std::string next_room;
        if (clockwise) {
            next_room = room_manager_.getNextRoomRight(current_room_);
        } else {
            next_room = room_manager_.getNextRoomLeft(current_room_);
        }
       
        waypoints_ = room_manager_.getPath(current_room_, next_room);
        current_waypoint_index_ = 0;
       
        if (!waypoints_.empty()) {
            target_x_ = waypoints_[0].x;
            target_y_ = waypoints_[0].y;
           
            RCLCPP_INFO(this->get_logger(), "🎯 %s → %s (%zu waypoints)",
                        current_room_.c_str(), next_room.c_str(), waypoints_.size());
            
            for (size_t i = 0; i < waypoints_.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "  WP%zu: (%.2f, %.2f)", 
                           i, waypoints_[i].x, waypoints_[i].y);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ No path found!");
            return;
        }
       
        current_room_ = next_room;
        state_ = State::ROTATING;
    }

    void controlLoop() {
        auto twist = geometry_msgs::msg::Twist();

        // Reactive obstacle handling
        if (state_ == State::MOVING_FORWARD) {
            if (is_path_blocked_) {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_vel_pub_->publish(twist);
                
                state_ = State::OBSTACLE_WAIT;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "⚠️  OBSTACLE! Stopping. Distance: %.2fm", min_obstacle_distance_);
                return;
            }
        }

        if (state_ == State::OBSTACLE_WAIT) {
            if (!is_path_blocked_) {
                RCLCPP_INFO(this->get_logger(), "✅ Path clear! Resuming...");
                state_ = State::MOVING_FORWARD;
            } else {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_vel_pub_->publish(twist);
                return;
            }
        }

        switch (state_) {
        case State::IDLE:
            break;

        case State::ROTATING:
            if (rotateToTarget()) {
                state_ = State::MOVING_FORWARD;
                RCLCPP_INFO(this->get_logger(), "↻ Aligned → Moving to (%.2f, %.2f)", 
                           target_x_, target_y_);
            }
            break;

        case State::MOVING_FORWARD:
            if (moveToTarget()) {
                state_ = State::ARRIVED;
                RCLCPP_INFO(this->get_logger(), "🏁 ARRIVED at %s", current_room_.c_str());
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_vel_pub_->publish(twist);
                state_ = State::IDLE;
            }
            break;

        case State::OBSTACLE_WAIT:
            break;

        case State::ARRIVED:
            break;
        }
    }

    bool rotateToTarget() {
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double target_angle = std::atan2(dy, dx);
        double angle_diff = normalizeAngle(target_angle - current_yaw_);
        
        auto twist = geometry_msgs::msg::Twist();

        if (std::abs(angle_diff) < 0.10) {  // 5.7 degrees tolerance
            twist.angular.z = 0.0;
            cmd_vel_pub_->publish(twist);
            return true;
        }

        // Slower rotation for real robot
        double angular_speed = 0.3;
        twist.angular.z = std::copysign(angular_speed, angle_diff);
        cmd_vel_pub_->publish(twist);
        return false;
    }

    bool moveToTarget() {
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
       
        auto twist = geometry_msgs::msg::Twist();
       
        // Larger threshold for real robots (wheel slip, imperfect odometry)
        if (distance < 0.15) {
            current_waypoint_index_++;
           
            if (current_waypoint_index_ >= waypoints_.size()) {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_vel_pub_->publish(twist);
                return true;
            } else {
                target_x_ = waypoints_[current_waypoint_index_].x;
                target_y_ = waypoints_[current_waypoint_index_].y;
                RCLCPP_INFO(this->get_logger(), "🔄 Waypoint %zu reached, next: (%.2f, %.2f)", 
                           current_waypoint_index_, target_x_, target_y_);
                state_ = State::ROTATING;
                return false;
            }
        }
       
        // Slower speed for real robots
        double base_speed = 0.08;  // Reduced from 0.12
        double linear_speed = base_speed;
        
        if (min_obstacle_distance_ < slow_distance_) {
            double slowdown_factor = (min_obstacle_distance_ - safe_distance_) / 
                                    (slow_distance_ - safe_distance_);
            slowdown_factor = std::max(0.3, std::min(1.0, slowdown_factor));
            linear_speed *= slowdown_factor;
        }
        
        twist.linear.x = linear_speed;
       
        // Heading correction
        double target_angle = std::atan2(dy, dx);
        double angle_diff = normalizeAngle(target_angle - current_yaw_);
        twist.angular.z = 0.4 * angle_diff;  // Gentler turning
       
        cmd_vel_pub_->publish(twist);
        return false;
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Member variables
    std::string robot_name_;
    std::string current_room_;
    real_maze_robots::RoomManager room_manager_;
    State state_;
    double current_x_, current_y_, current_yaw_;
    double target_x_, target_y_;
    double min_obstacle_distance_;
    bool is_path_blocked_;
    double safe_distance_;
    double slow_distance_;

    std::vector<real_maze_robots::Position> waypoints_;
    size_t current_waypoint_index_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr left_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr right_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealRobotController>());
    rclcpp::shutdown();
    return 0;
}
