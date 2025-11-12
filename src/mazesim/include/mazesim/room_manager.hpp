#ifndef MAZESIM_ROOM_MANAGER_HPP
#define MAZESIM_ROOM_MANAGER_HPP

#include <string>
#include <map>
#include <array>
#include <vector>

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
        // Define room positions (centers) - keep your originals, as waypoints build on them
        rooms_["room_1"] = {0.0, 0.0, 0.0, 1.57};      // Upper-left
        rooms_["room_2"] = {-1.2, 0.0, 0.0, 3.14};     // Upper-right
        rooms_["room_3"] = {-1.2, 1.0, 0.0, 0.0};      // Lower-left
        rooms_["room_4"] = {-0.2, 0.8, 0.0, -1.57};    // Lower-right
        
        // Define clockwise (right) rotation pattern - no change
        right_pattern_["room_1"] = "room_2";
        right_pattern_["room_2"] = "room_3";
        right_pattern_["room_3"] = "room_4";
        right_pattern_["room_4"] = "room_1";
        
        // Define counter-clockwise (left) rotation pattern - no change
        left_pattern_["room_1"] = "room_4";
        left_pattern_["room_4"] = "room_3";
        left_pattern_["room_3"] = "room_2";
        left_pattern_["room_2"] = "room_1";
        
        // Define paths with your new waypoints (intermediates only; final room added in getPath)
        // Clockwise (right) transitions
        paths_["room_1_room_2"] = { {-0.8, -0.2}, {-0.8, -0.6}, {-1.4, -0.4} };
        paths_["room_2_room_3"] = { {-1.2, -0.5}, {-0.8, -0.4}, {-0.38, -0.4}, {-0.7, 0.8}, {-1.4, 0.6}, {-1.4, 1.00}, {-1.4, 1.20} };
        paths_["room_3_room_4"] = { {-1.4, 0.8}, {-1.3, 0.4}, {-0.7, 0.6}, {-0.5, 0.6}, {-0.6, 1.3}, {-0.1, 1.27}, {-0.25, 0.7} };
        paths_["room_4_room_1"] = { {-0.3, 1.3}, {-0.3, 1.35}, {0.1, 1.35}, {0.4, 1.2}, {0.3, 0.2}, {-0.1, 0.3}, {-0.8, 0.25}, {-0.8, 0.0}, {-0.6, -0.2}, {-.45, -0.2}, {-0.2, -0.12} };
        
        // Counter-clockwise (left) transitions
        // paths_["room_1_room_4"] = { {-0.7, -0.1}, {-0.7, 0.3}, {0.2, 0.3}, {0.2, 1.3}, {-0.2, 1.3} };
        // paths_["room_2_room_1"] = { {-1.2, 0.5}, {-0.7, 0.5}, {-0.7, -0.1} };
        // paths_["room_3_room_2"] = { {-1.3, 0.8}, {-0.7, 0.8}, {-0.7, 0.5}, {-1.2, 0.5} };
        // paths_["room_4_room_3"] = { {-0.2, 1.3}, {-0.7, 1.3}, {-0.7, 0.8}, {-1.3, 0.8} };


        // Counter-clockwise (left) transitions
        paths_["room_1_room_4"] = { {-0.2, -0.12}, {-.45, -0.2}, {-0.5, -0.2}, {-0.7, 0.0}, {-0.7, 0.25}, {-0.7, 0.5}, {0.4, 0.36}, {0.46, 0.4}, {0.3, 1.2}, {0.1, 1.35}, {-0.3, 1.35}, {-0.3, 1.3} };

        paths_["room_4_room_3"] = { {-0.25, 0.7}, {-0.1, 1.27}, {-0.6, 1.3}, {-0.5, 0.6}, {-0.7, 0.6}, {-1.3, 0.4}, {-1.4, 0.8} };

        paths_["room_3_room_2"] = { {-1.4, 1.20}, {-1.4, 1.00}, {-1.4, 0.6}, {-0.7, 0.8}, {-0.38, -0.4}, {-0.8, -0.4}, {-1.2, -0.5} };

        paths_["room_2_room_1"] = { {-1.4, -0.4}, {-0.8, -0.6}, {-0.8, -0.2} };
    }
    
    Position getRoomPosition(const std::string& room_name) const {
        auto it = rooms_.find(room_name);
        if (it != rooms_.end()) {
            return it->second;
        }
        return {0.0, 0.0, 0.0, 0.0};
    }
    
    std::string getNextRoomRight(const std::string& current_room) const {
        auto it = right_pattern_.find(current_room);
        if (it != right_pattern_.end()) {
            return it->second;
        }
        return current_room;
    }
    
    std::string getNextRoomLeft(const std::string& current_room) const {
        auto it = left_pattern_.find(current_room);
        if (it != left_pattern_.end()) {
            return it->second;
        }
        return current_room;
    }
    
    // NEW: Get waypoints for path between rooms
    std::vector<Position> getPath(const std::string& from_room, const std::string& to_room) const {
        std::string path_key = from_room + "_" + to_room;
        auto it = paths_.find(path_key);
        
        std::vector<Position> waypoints;
        if (it != paths_.end()) {
            for (const auto& wp : it->second) {
                waypoints.push_back({wp.x, wp.y, 0.0, 0.0});
            }
        }
        
        // Add final destination
        waypoints.push_back(getRoomPosition(to_room));
        return waypoints;
    }
    
    std::array<std::string, 4> getAllRoomNames() const {
        return {"room_1", "room_2", "room_3", "room_4"};
    }

private:
    struct Waypoint {
        double x;
        double y;
    };
    
    std::map<std::string, Position> rooms_;
    std::map<std::string, std::string> right_pattern_;
    std::map<std::string, std::string> left_pattern_;
    std::map<std::string, std::vector<Waypoint>> paths_;  // NEW
};

} // namespace mazesim

#endif // MAZESIM_ROOM_MANAGER_HPP