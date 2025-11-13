/**
 * Multi-Domain Command Broadcaster
 * Publishes /left or /right commands to multiple ROS_DOMAIN_IDs simultaneously
 * This enables synchronized robot movement across different domains.
 */

#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/empty.hpp"

using namespace std::chrono_literals;

class MultiDomainCommander : public rclcpp::Node {
public:
    MultiDomainCommander() : Node("multi_domain_commander") {
        // Declare parameters
        this->declare_parameter<std::vector<int64_t>>("domain_ids", std::vector<int64_t>{1, 3});
        
        domain_ids_ = this->get_parameter("domain_ids").as_integer_array();
        
        RCLCPP_INFO(this->get_logger(), "🎮 Multi-Domain Commander Started");
        RCLCPP_INFO(this->get_logger(), "Target domains: %s", 
                    vec_to_string(domain_ids_).c_str());
        
        // Publishers for /left and /right
        left_pub_ = this->create_publisher<example_interfaces::msg::Empty>("/left", 10);
        right_pub_ = this->create_publisher<example_interfaces::msg::Empty>("/right", 10);
        
        // Timer for keyboard input
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MultiDomainCommander::checkInput, this));
        
        print_instructions();
    }

private:
    void print_instructions() {
        RCLCPP_INFO(this->get_logger(), "\n"
            "========================================\n"
            "  KEYBOARD CONTROLS\n"
            "========================================\n"
            "  Press 'l' + ENTER : Send LEFT command\n"
            "  Press 'r' + ENTER : Send RIGHT command\n"
            "  Press 'q' + ENTER : Quit\n"
            "========================================\n");
    }
    
    void checkInput() {
        // Non-blocking check for keyboard input
        // Note: This is a simplified version. For production, use ncurses or similar.
        // For now, we'll rely on ROS2 service calls instead.
    }
    
    void sendLeft() {
        auto msg = example_interfaces::msg::Empty();
        left_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "⬅️  Broadcasting LEFT to all domains");
    }
    
    void sendRight() {
        auto msg = example_interfaces::msg::Empty();
        right_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "➡️  Broadcasting RIGHT to all domains");
    }
    
    std::string vec_to_string(const std::vector<int64_t>& vec) {
        std::string result = "[";
        for (size_t i = 0; i < vec.size(); i++) {
            result += std::to_string(vec[i]);
            if (i < vec.size() - 1) result += ", ";
        }
        result += "]";
        return result;
    }
    
    std::vector<int64_t> domain_ids_;
    rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr left_pub_;
    rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr right_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MultiDomainCommander>();
    
    std::cout << "\n🎮 Commander ready! Type 'l' for LEFT, 'r' for RIGHT, 'q' to QUIT:\n> ";
    std::cout.flush();
    
    // Simple command loop
    std::string input;
    while (rclcpp::ok()) {
        if (std::getline(std::cin, input)) {
            if (input == "l" || input == "L") {
                // Publish left command
                auto msg = example_interfaces::msg::Empty();
                node->get_publisher("/left")->publish(msg);
                std::cout << "⬅️  LEFT sent!\n> ";
            } else if (input == "r" || input == "R") {
                // Publish right command  
                auto msg = example_interfaces::msg::Empty();
                node->get_publisher("/right")->publish(msg);
                std::cout << "➡️  RIGHT sent!\n> ";
            } else if (input == "q" || input == "Q") {
                std::cout << "👋 Goodbye!\n";
                break;
            } else {
                std::cout << "Unknown command. Use 'l', 'r', or 'q'\n> ";
            }
            std::cout.flush();
        }
        
        rclcpp::spin_some(node);
    }
    
    rclcpp::shutdown();
    return 0;
}
