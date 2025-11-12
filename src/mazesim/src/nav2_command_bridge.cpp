#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"  // <-- ADD THIS LINE
#include "mazesim/room_manager.hpp"


using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Nav2CommandBridge : public rclcpp::Node {
public:
    Nav2CommandBridge() : Node("nav2_command_bridge") {
        // Declare parameters
        this->declare_parameter<std::string>("robot_name", "robot1");
        this->declare_parameter<std::string>("initial_room", "room_1");

        // Get parameters
        robot_name_ = this->get_parameter("robot_name").as_string();
        current_room_ = this->get_parameter("initial_room").as_string();

        RCLCPP_INFO(this->get_logger(), "Starting Nav2 bridge for %s in %s",
                    robot_name_.c_str(), current_room_.c_str());

        // Action client for Nav2
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "/" + robot_name_ + "/navigate_to_pose");

        // Subscribers for left/right commands
        left_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/left", 10,
            std::bind(&Nav2CommandBridge::leftCallback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<example_interfaces::msg::Empty>(
            "/right", 10,
            std::bind(&Nav2CommandBridge::rightCallback, this, std::placeholders::_1));

        // Wait for action server
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        if (!nav_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 action server connected. Ready for commands.");
        }

        is_navigating_ = false;
    }

private:
    void leftCallback(const example_interfaces::msg::Empty::SharedPtr /*msg*/) {
        if (!is_navigating_) {
            RCLCPP_INFO(this->get_logger(), "Received LEFT command");
            navigateToNextRoom(false); // counter-clockwise
        } else {
            RCLCPP_WARN(this->get_logger(), "Already navigating, ignoring LEFT command");
        }
    }

    void rightCallback(const example_interfaces::msg::Empty::SharedPtr /*msg*/) {
        if (!is_navigating_) {
            RCLCPP_INFO(this->get_logger(), "Received RIGHT command");
            navigateToNextRoom(true); // clockwise
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

        // Get path with waypoints
        auto waypoints = room_manager_.getPath(current_room_, next_room);
        
        if (waypoints.empty()) {
            RCLCPP_WARN(this->get_logger(), "No waypoints found from %s to %s",
                        current_room_.c_str(), next_room.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Moving from %s to %s via %zu waypoints",
                    current_room_.c_str(), next_room.c_str(), waypoints.size());

        // Send goal to Nav2 (final destination)
        sendNav2Goal(waypoints.back(), next_room);
    }

    void sendNav2Goal(const mazesim::Position& target, const std::string& target_room) {
        if (!nav_client_->wait_for_action_server(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
            return;
        }

        // Create goal message
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = target.x;
        goal_msg.pose.pose.position.y = target.y;
        goal_msg.pose.pose.position.z = target.z;

        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, target.yaw);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f, yaw=%.2f",
                    target.x, target.y, target.yaw);

        // Send goal with callbacks
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&Nav2CommandBridge::goalResponseCallback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&Nav2CommandBridge::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&Nav2CommandBridge::resultCallback, this, std::placeholders::_1, target_room);

        is_navigating_ = true;
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(const GoalHandleNavigate::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            is_navigating_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, navigating...");
        }
    }

    void feedbackCallback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        // Log distance remaining periodically
        static int counter = 0;
        if (counter++ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f meters",
                        feedback->distance_remaining);
        }
    }

    void resultCallback(
        const GoalHandleNavigate::WrappedResult& result,
        const std::string& target_room) {
        is_navigating_ = false;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Successfully arrived at %s!", 
                            target_room.c_str());
                current_room_ = target_room;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    // Member variables
    std::string robot_name_;
    std::string current_room_;
    bool is_navigating_;
    mazesim::RoomManager room_manager_;

    // ROS interfaces
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr left_sub_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr right_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2CommandBridge>());
    rclcpp::shutdown();
    return 0;
}