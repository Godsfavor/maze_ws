#ifndef MAZESIM_ROOM_MANAGER_HPP
#define MAZESIM_ROOM_MANAGER_HPP

#include <string>
#include <map>
#include <array>

namespace mazesim {

struct Position {
    double x;
    double y;
    double z;
    double yaw;
};

class RoomManager {
public:
    RoomManager() {
        // Define room positions (from config file)
        rooms_["room_1"] = {3.81, 16.10, 0.0, 0.0};
        rooms_["room_2"] = {16.31, 16.10, 0.0, 0.0};
        rooms_["room_3"] = {3.81, 3.60, 0.0, 0.0};
        rooms_["room_4"] = {16.31, 3.60, 0.0, 0.0};
        
        // Define clockwise (right) rotation pattern
        right_pattern_["room_1"] = "room_2";
        right_pattern_["room_2"] = "room_4";
        right_pattern_["room_4"] = "room_3";
        right_pattern_["room_3"] = "room_1";
        
        // Define counter-clockwise (left) rotation pattern
        left_pattern_["room_1"] = "room_3";
        left_pattern_["room_3"] = "room_4";
        left_pattern_["room_4"] = "room_2";
        left_pattern_["room_2"] = "room_1";
    }
    
    // Get position of a room by name
    Position getRoomPosition(const std::string& room_name) const {
        auto it = rooms_.find(room_name);
        if (it != rooms_.end()) {
            return it->second;
        }
        // Return origin if room not found
        return {0.0, 0.0, 0.0, 0.0};
    }
    
    // Get next room for right (clockwise) movement
    std::string getNextRoomRight(const std::string& current_room) const {
        auto it = right_pattern_.find(current_room);
        if (it != right_pattern_.end()) {
            return it->second;
        }
        return current_room; // Stay in same room if not found
    }
    
    // Get next room for left (counter-clockwise) movement
    std::string getNextRoomLeft(const std::string& current_room) const {
        auto it = left_pattern_.find(current_room);
        if (it != left_pattern_.end()) {
            return it->second;
        }
        return current_room; // Stay in same room if not found
    }
    
    // Get all room names
    std::array<std::string, 4> getAllRoomNames() const {
        return {"room_1", "room_2", "room_3", "room_4"};
    }

private:
    std::map<std::string, Position> rooms_;
    std::map<std::string, std::string> right_pattern_;
    std::map<std::string, std::string> left_pattern_;
};

} // namespace mazesim

#endif // MAZESIM_ROOM_MANAGER_HPP
