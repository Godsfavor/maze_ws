#include <iostream>
#include <iomanip>
#include "mazesim/room_config.hpp"

int main() {
    mazesim::RoomConfig config;

    std::cout << "=== Room Position Configuration ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    // Display all room positions
    for (int i = 0; i < 4; ++i) {
        std::string room_id = "room_" + std::to_string(i);
        auto room = config.getRoom(room_id);
        
        std::cout << "\n" << room.name << " (" << room_id << "):" << std::endl;
        std::cout << "  Position: (" << room.x << ", " << room.y << ", " << room.z << ")" << std::endl;
        std::cout << "  Orientation: " << room.yaw << " rad" << std::endl;
    }

    // Test clockwise rotation
    std::cout << "\n=== Clockwise Rotation (Right Signal) ===" << std::endl;
    std::string current = "room_0";
    for (int i = 0; i < 4; ++i) {
        std::string next = config.getNextRoom(current, true);
        std::cout << current << " → " << next << std::endl;
        current = next;
    }

    // Test counter-clockwise rotation
    std::cout << "\n=== Counter-Clockwise Rotation (Left Signal) ===" << std::endl;
    current = "room_0";
    for (int i = 0; i < 4; ++i) {
        std::string next = config.getNextRoom(current, false);
        std::cout << current << " → " << next << std::endl;
        current = next;
    }

    // Test initial assignments
    std::cout << "\n=== Initial Robot Assignments ===" << std::endl;
    for (int robot_id = 1; robot_id <= 4; ++robot_id) {
        std::string room = config.getInitialRoom(robot_id);
        auto pos = config.getRoom(room);
        std::cout << "Robot " << robot_id << " → " << pos.name 
                  << " at (" << pos.x << ", " << pos.y << ")" << std::endl;
    }

    std::cout << "\n=== Movement Parameters ===" << std::endl;
    std::cout << "Linear Speed: " << mazesim::RoomConfig::LINEAR_SPEED << " m/s" << std::endl;
    std::cout << "Angular Speed: " << mazesim::RoomConfig::ANGULAR_SPEED << " rad/s" << std::endl;
    std::cout << "Position Tolerance: " << mazesim::RoomConfig::POSITION_TOLERANCE << " m" << std::endl;
    std::cout << "Angle Tolerance: " << mazesim::RoomConfig::ANGLE_TOLERANCE << " rad" << std::endl;

    return 0;
}