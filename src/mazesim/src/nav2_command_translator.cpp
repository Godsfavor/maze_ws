#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mazesim/room_manager.hpp"


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Nav2CommandTranslator : public rclcpp::Node {
public:
    Nav2CommandTranslator() : Node("nav2_command_translator") {
        // Declare parameters
        this->declare_parameter<std::string>("robot_name", "robot1");
        this->declare_parameter<std::string>("initial_room", "room_1");
        
        // Get parameters
        robot_name_ = this->get_parameter("robot_name").as_string();
        current_room_ = this->get_parameter("initial_room").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Starting Nav2 command translator for %s in %s", 
                    robot_name_.c_str(), current_room_.c_str());
        
        // Create action client for Nav2
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "/" + robot_name_ + "/navigate_to_pose");
        
        // Subscribe to left/right commands
        left_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/left", 10,
            std::bind(&Nav2CommandTranslator::leftCallback, this, std::placeholders::_1));
        
        right_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/right", 10,
            std::bind(&Nav2CommandTranslator::rightCallback, this, std::placeholders::_1));
        
        // Wait for action server
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 action server connected!");
        }
        
        is_navigating_ = false;
    }

private:
    void leftCallback(const example_interfaces::msg::Empty::SharedPtr msg) {
        (void)msg;  // Unused
        if (!is_navigating_) {
            RCLCPP_INFO(this->get_logger(), "Received LEFT command");
            navigateToNextRoom(false);  // Counter-clockwise
        } else {
            RCLCPP_WARN(this->get_logger(), "Already navigating, ignoring LEFT command");
        }
    }
    
    void rightCallback(const example_interfaces::msg::Empty::SharedPtr msg) {
        (void)msg;  // Unused
        if (!is_navigating_) {
            RCLCPP_INFO(this->get_logger(), "Received RIGHT command");
            navigateToNextRoom(true);  // Clockwise
        } else {
            RCLCPP_WARN(this->get_logger(), "Already navigating, ignoring RIGHT command");
        }
    }
    
    void navigateToNextRoom(bool clockwise) {
        // Determine next room
        std::string next_room;
        if (clockwise) {
            next_room = room_manager_.getNextRoomRight(current_room_);
        } else {
            next_room = room_manager_.getNextRoomLeft(current_room_);
        }
        
        // Get target position
        auto target_pos = room_manager_.getRoomPosition(next_room);
        
        RCLCPP_INFO(this->get_logger(), "Navigating from %s to %s (%.2f, %.2f)", 
                    current_room_.c_str(), next_room.c_str(), 
                    target_pos.x, target_pos.y);
        
        // Create Nav2 goal
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = target_pos.x;
        goal_msg.pose.pose.position.y = target_pos.y;
        goal_msg.pose.pose.position.z = 0.0;
        
        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, target_pos.yaw);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();
        
        // Send goal with callbacks
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&Nav2CommandTranslator::goalResponseCallback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&Nav2CommandTranslator::feedbackCallback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&Nav2CommandTranslator::resultCallback, this, std::placeholders::_1);
        
        is_navigating_ = true;
        next_room_ = next_room;
        
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    void goalResponseCallback(const GoalHandleNavigate::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            is_navigating_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, navigating...");
        }
    }
    
    void feedbackCallback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // Optional: print progress
        // RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", 
        //             feedback->distance_remaining);
    }
    
    void resultCallback(const GoalHandleNavigate::WrappedResult & result) {
        is_navigating_ = false;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded! Now in %s", 
                           next_room_.c_str());
                current_room_ = next_room_;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
    
    // Member variables
    std::string robot_name_;
    std::string current_room_;
    std::string next_room_;
    mazesim::RoomManager room_manager_;
    bool is_navigating_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr left_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr right_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2CommandTranslator>());
    rclcpp::shutdown();
    return 0;
}