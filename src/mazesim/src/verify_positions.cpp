#include <iostream>
#include <iomanip>
#include "mazesim/room_manager.hpp"

int main() {
    mazesim::RoomManager manager;
    
    std::cout << "=== Room Position Verification ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    
    auto rooms = manager.getAllRoomNames();
    
    std::cout << "\n--- Room Positions ---" << std::endl;
    for (const auto& room_name : rooms) {
        auto pos = manager.getRoomPosition(room_name);
        std::cout << room_name << ": "
                  << "x=" << pos.x << "m, "
                  << "y=" << pos.y << "m, "
                  << "yaw=" << pos.yaw << " rad" << std::endl;
    }
    
    std::cout << "\n--- Right (Clockwise) Pattern ---" << std::endl;
    for (const auto& room_name : rooms) {
        std::string next = manager.getNextRoomRight(room_name);
        std::cout << room_name << " -> " << next << std::endl;
    }
    
    std::cout << "\n--- Left (Counter-clockwise) Pattern ---" << std::endl;
    for (const auto& room_name : rooms) {
        std::string next = manager.getNextRoomLeft(room_name);
        std::cout << room_name << " -> " << next << std::endl;
    }
    
    std::cout << "\n--- Distance Between Rooms ---" << std::endl;
    auto pos1 = manager.getRoomPosition("room_1");
    auto pos2 = manager.getRoomPosition("room_2");
    double dist_horizontal = std::abs(pos2.x - pos1.x);
    
    auto pos3 = manager.getRoomPosition("room_3");
    double dist_vertical = std::abs(pos1.y - pos3.y);
    
    std::cout << "Horizontal distance: " << dist_horizontal << "m" << std::endl;
    std::cout << "Vertical distance: " << dist_vertical << "m" << std::endl;
    
    return 0;
}
