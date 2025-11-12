#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mazesim/room_manager.hpp"

using namespace std::chrono_literals;

class ReactiveRobotController : public rclcpp::Node {
public:
    ReactiveRobotController() : Node("reactive_robot_controller") {
        // Parameters
        this->declare_parameter<std::string>("robot_name", "robot1");
        this->declare_parameter<std::string>("initial_room", "room_1");
        this->declare_parameter<double>("safe_distance", 0.15);  // Stop if obstacle closer than this
        this->declare_parameter<double>("slow_distance", 0.20);   // Slow down if obstacle closer than this

        robot_name_ = this->get_parameter("robot_name").as_string();
        current_room_ = this->get_parameter("initial_room").as_string();
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        slow_distance_ = this->get_parameter("slow_distance").as_double();

        RCLCPP_INFO(this->get_logger(), "🤖 Starting %s in %s", 
                    robot_name_.c_str(), current_room_.c_str());

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robot_name_ + "/cmd_vel", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot_name_ + "/odom", 10,
            std::bind(&ReactiveRobotController::odomCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/" + robot_name_ + "/scan", 10,
            std::bind(&ReactiveRobotController::scanCallback, this, std::placeholders::_1));

        left_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/left", 10,
            std::bind(&ReactiveRobotController::leftCallback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/right", 10,
            std::bind(&ReactiveRobotController::rightCallback, this, std::placeholders::_1));

        // Timer for control loop
        timer_ = this->create_wall_timer(
            50ms, std::bind(&ReactiveRobotController::controlLoop, this));

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
    }

private:
    enum class State {
        IDLE,
        ROTATING,
        MOVING_FORWARD,
        OBSTACLE_WAIT,  // NEW: waiting for obstacle to clear
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
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check NARROW front cone (±15 degrees)
        int total_rays = msg->ranges.size();
        int cone_width = total_rays / 12;  // 30 degrees total
        
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

        // REACTIVE: Update obstacle status
        is_path_blocked_ = (min_obstacle_distance_ < safe_distance_);
    }

    void leftCallback(const example_interfaces::msg::Empty::SharedPtr /*msg*/) {
        if (state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "⬅️  LEFT signal received");
            startMovement(false);  // counter-clockwise
        }
    }

    void rightCallback(const example_interfaces::msg::Empty::SharedPtr /*msg*/) {
        if (state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "➡️  RIGHT signal received");
            startMovement(true);   // clockwise
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
            
            // Print all waypoints for debugging
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

        // REACTIVE BEHAVIOR: Check for obstacles when moving
        if (state_ == State::MOVING_FORWARD) {
            if (is_path_blocked_) {
                // STOP immediately
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_vel_pub_->publish(twist);
                
                state_ = State::OBSTACLE_WAIT;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "⚠️  OBSTACLE! Stopping. Distance: %.2fm", min_obstacle_distance_);
                return;
            }
        }

        // If we were waiting for obstacle to clear
        if (state_ == State::OBSTACLE_WAIT) {
            if (!is_path_blocked_) {
                RCLCPP_INFO(this->get_logger(), "✅ Path clear! Resuming...");
                state_ = State::MOVING_FORWARD;
            } else {
                // Still blocked, keep waiting
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
                // Reached final destination
                state_ = State::ARRIVED;
                RCLCPP_INFO(this->get_logger(), "🏁 ARRIVED at %s", current_room_.c_str());
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_vel_pub_->publish(twist);
                state_ = State::IDLE;
            }
            break;

        case State::OBSTACLE_WAIT:
            // Handled above
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

        if (std::abs(angle_diff) < 0.15) {  // Tighter tolerance
            twist.angular.z = 0.0;
            cmd_vel_pub_->publish(twist);
            return true;
        }

        // Smooth rotation
        double angular_speed = 0.4;
        twist.angular.z = std::copysign(angular_speed, angle_diff);
        cmd_vel_pub_->publish(twist);
        return false;
    }

    bool moveToTarget() {
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
       
        auto twist = geometry_msgs::msg::Twist();
       
        // Check if waypoint reached
        if (distance < 0.25) {  // Smaller threshold for better accuracy
            current_waypoint_index_++;
           
            if (current_waypoint_index_ >= waypoints_.size()) {
                // All waypoints completed
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_vel_pub_->publish(twist);
                return true;
            } else {
                // Move to next waypoint
                target_x_ = waypoints_[current_waypoint_index_].x;
                target_y_ = waypoints_[current_waypoint_index_].y;
                RCLCPP_INFO(this->get_logger(), "📍 Waypoint %zu reached, next: (%.2f, %.2f)", 
                           current_waypoint_index_, target_x_, target_y_);
                state_ = State::ROTATING;  // Re-align to new waypoint
                return false;
            }
        }
       
        // REACTIVE: Adjust speed based on obstacle distance
        double base_speed = 0.12;
        double linear_speed = base_speed;
        
        if (min_obstacle_distance_ < slow_distance_) {
            // Slow down as we approach obstacles
            double slowdown_factor = (min_obstacle_distance_ - safe_distance_) / 
                                    (slow_distance_ - safe_distance_);
            slowdown_factor = std::max(0.3, std::min(1.0, slowdown_factor));
            linear_speed *= slowdown_factor;
            
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Slowing down: %.2f m/s (obstacle at %.2fm)", linear_speed, min_obstacle_distance_);
        }
        
        twist.linear.x = linear_speed;
       
        // Heading correction (proportional control)
        double target_angle = std::atan2(dy, dx);
        double angle_diff = normalizeAngle(target_angle - current_yaw_);
        twist.angular.z = 0.5 * angle_diff;  // Smooth turning
       
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
    mazesim::RoomManager room_manager_;
    State state_;
    double current_x_, current_y_, current_yaw_;
    double target_x_, target_y_;
    double min_obstacle_distance_;
    bool is_path_blocked_;
    double safe_distance_;   // Stop threshold
    double slow_distance_;   // Slow down threshold

    std::vector<mazesim::Position> waypoints_;
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
    rclcpp::spin(std::make_shared<ReactiveRobotController>());
    rclcpp::shutdown();
    return 0;
}