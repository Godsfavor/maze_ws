#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mazesim/room_manager.hpp"

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        // Declare parameters
        this->declare_parameter<std::string>("robot_name", "robot1");
        this->declare_parameter<std::string>("initial_room", "room_1");
        
        // Get parameters
        robot_name_ = this->get_parameter("robot_name").as_string();
        current_room_ = this->get_parameter("initial_room").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Starting controller for %s in %s", 
                    robot_name_.c_str(), current_room_.c_str());
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robot_name_ + "/cmd_vel", 10);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot_name_ + "/odom", 10,
            std::bind(&RobotController::odomCallback, this, std::placeholders::_1));
        
        left_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/left", 10,
            std::bind(&RobotController::leftCallback, this, std::placeholders::_1));
        
        right_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/right", 10,
            std::bind(&RobotController::rightCallback, this, std::placeholders::_1));
        
        // Timer for control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotController::controlLoop, this));
        
        // Initialize state
        state_ = State::IDLE;
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        target_x_ = 0.0;
        target_y_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Robot controller initialized and waiting for signals");
    }

private:
    enum class State {
        IDLE,
        ROTATING,
        MOVING_FORWARD,
        ARRIVED
    };
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Update current position
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
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
    
    void leftCallback(const example_interfaces::msg::Empty::SharedPtr msg) {
        if (state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "Received LEFT signal");
            startMovement(false);  // false = left (counter-clockwise)
        }
    }
    
    void rightCallback(const example_interfaces::msg::Empty::SharedPtr msg) {
        if (state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "Received RIGHT signal");
            startMovement(true);  // true = right (clockwise)
        }
    }
    
    void startMovement(bool clockwise) {
        // Determine next room
        std::string next_room;
        if (clockwise) {
            next_room = room_manager_.getNextRoomRight(current_room_);
        } else {
            next_room = room_manager_.getNextRoomLeft(current_room_);
        }
        
        // Get target position
        auto target_pos = room_manager_.getRoomPosition(next_room);
        target_x_ = target_pos.x;
        target_y_ = target_pos.y;
        
        RCLCPP_INFO(this->get_logger(), "Moving from %s to %s (%.2f, %.2f)", 
                    current_room_.c_str(), next_room.c_str(), target_x_, target_y_);
        
        // Update state
        current_room_ = next_room;
        state_ = State::ROTATING;
    }
    
    void controlLoop() {
        auto twist = geometry_msgs::msg::Twist();
        
        switch (state_) {
            case State::IDLE:
                // Do nothing, wait for signal
                break;
                
            case State::ROTATING:
                if (rotateToTarget()) {
                    // Rotation complete, start moving forward
                    state_ = State::MOVING_FORWARD;
                    RCLCPP_INFO(this->get_logger(), "Rotation complete, moving forward");
                }
                break;
                
            case State::MOVING_FORWARD:
                if (moveToTarget()) {
                    // Arrived at target
                    state_ = State::ARRIVED;
                    RCLCPP_INFO(this->get_logger(), "Arrived at target room");
                    
                    // Stop the robot
                    cmd_vel_pub_->publish(twist);
                    
                    // Return to idle after a short delay
                    state_ = State::IDLE;
                }
                break;
                
            case State::ARRIVED:
                // Transition handled in MOVING_FORWARD
                break;
        }
    }
    
    bool rotateToTarget() {
        // Calculate angle to target
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double target_angle = std::atan2(dy, dx);
        
        // Calculate angle difference
        double angle_diff = normalizeAngle(target_angle - current_yaw_);
        
        auto twist = geometry_msgs::msg::Twist();
        
        // Angle tolerance: 0.1 radians (~5.7 degrees)
        if (std::abs(angle_diff) < 0.1) {
            // Rotation complete
            twist.angular.z = 0.0;
            cmd_vel_pub_->publish(twist);
            return true;
        }
        
        // Rotate towards target
        double angular_speed = 0.5;  // rad/s
        if (angle_diff > 0) {
            twist.angular.z = angular_speed;
        } else {
            twist.angular.z = -angular_speed;
        }
        
        cmd_vel_pub_->publish(twist);
        return false;
    }
    
    bool moveToTarget() {
        // Calculate distance to target
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        auto twist = geometry_msgs::msg::Twist();
        
        // Position tolerance: 0.2 meters
        if (distance < 0.2) {
            // Arrived
            twist.linear.x = 0.0;
            cmd_vel_pub_->publish(twist);
            return true;
        }
        
        // Move forward
        double linear_speed = 0.15;  // m/s (slow and steady)
        twist.linear.x = linear_speed;
        
        // Minor heading correction
        double target_angle = std::atan2(dy, dx);
        double angle_diff = normalizeAngle(target_angle - current_yaw_);
        twist.angular.z = 0.3 * angle_diff;  // Proportional control
        
        cmd_vel_pub_->publish(twist);
        return false;
    }
    
    double normalizeAngle(double angle) {
        // Normalize angle to [-pi, pi]
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
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr left_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr right_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}