#ifndef REAL_MAZE_ROBOTS_ROOM_MANAGER_HPP
#define REAL_MAZE_ROBOTS_ROOM_MANAGER_HPP
#include <string>
#include <map>
#include <vector>
namespace real_maze_robots {
struct Position {
    double x;
    double y;
    double yaw;
};
class RoomManager {
public:
    RoomManager() {
        // TODO: Replace these with YOUR real measurements!
        // These are PLACEHOLDERS - measure your actual lab layout
        // rooms_["room_1"] = {0.0, 0.0, 1.57}; // Start position
        // rooms_["room_2"] = {2.0, 0.0, 3.14}; // Example: 2m to the right
        // rooms_["room_3"] = {2.0, 2.0, 0.0}; // Example: 2m right, 2m forward
        // rooms_["room_4"] = {0.0, 2.0, -1.57}; // Example: back to left, 2m forward
        // Room positions (measured from map)
        rooms_["room_1"] = {0.01, -0.10, 1.57};
        rooms_["room_2"] = {-1.14, 0.00, 3.14};
        rooms_["room_3"] = {-1.24, 1.10, 0.00};
        rooms_["room_4"] = {-0.24, 0.85, -1.57};
        // Movement patterns (no change)
        right_pattern_["room_1"] = "room_2";
        right_pattern_["room_2"] = "room_3";
        right_pattern_["room_3"] = "room_4";
        right_pattern_["room_4"] = "room_1";



        left_pattern_["room_1"] = "room_4";
        left_pattern_["room_4"] = "room_3";
        left_pattern_["room_3"] = "room_2";
        left_pattern_["room_2"] = "room_1";


        // TODO: Add real waypoints after measuring your lab
        // For now, direct paths (no intermediate waypoints)
        // You'll add these after testing basic movement
        paths_["room_1_room_2"] = { {-0.24, -0.05}, {-0.54, -0.05}, {-0.59, -0.40}, {-0.94, -0.40}, {-1.29, -0.35}, {-1.34, 0.00} };
        paths_["room_2_room_3"] = { {-1.09, -0.05}, {-0.94, -0.20}, {-0.74, -0.25}, {-0.64, 0.10}, {-0.74, 0.70}, {-1.39, 0.70}, {-1.34, 1.00} };
        paths_["room_3_room_4"] = { {-1.24, 0.90}, {-1.19, 0.65}, {-0.84, 0.60}, {-0.69, 0.85}, {-0.69, 1.20}, {-0.39, 1.30}, {-0.34, 1.00} };
        paths_["room_4_room_1"] = { {-0.29, 1.10}, {-0.14, 1.30}, {0.21, 1.30}, {0.26, 0.85}, {0.26, 0.45}, {-0.24, 0.40}, {-0.64, 0.40}, {-0.64, 0.05}, {-0.59, -0.10}, {-0.29, 0.00} };
        
        
        
        paths_["room_1_room_4"] = { {-0.29, -0.05}, {-0.59, 0.00}, {-0.64, 0.35}, {-0.24, 0.35}, {0.26, 0.45}, {0.26, 0.85}, {0.21, 1.35}, {-0.34, 1.30}, {-0.29, 0.95} };
        paths_["room_4_room_3"] = { {-0.34, 1.15}, {-0.64, 1.30}, {-0.69, 1.00}, {-0.69, 0.70}, {-1.14, 0.70}, {-1.34, 0.70}, {-1.34, 1.00} };
        paths_["room_3_room_2"] = { {-1.34, 0.75}, {-1.09, 0.60}, {-0.74, 0.65}, {-0.69, 0.30}, {-0.64, 0.00}, {-0.64, -0.30}, {-0.99, -0.30}, {-1.24, -0.30}, {-1.29, 0.05} };
        paths_["room_2_room_1"] = { {-1.09, -0.35}, {-0.59, -0.35}, {-0.59, -0.05}, {-0.19, 0.00} };
    }
    Position getRoomPosition(const std::string& room_name) const {
        auto it = rooms_.find(room_name);
        if (it != rooms_.end()) {
            return it->second;
        }
        return {0.0, 0.0, 0.0};
    }
    std::string getNextRoomRight(const std::string& current_room) const {
        auto it = right_pattern_.find(current_room);
        return (it != right_pattern_.end()) ? it->second : current_room;
    }
    std::string getNextRoomLeft(const std::string& current_room) const {
        auto it = left_pattern_.find(current_room);
        return (it != left_pattern_.end()) ? it->second : current_room;
    }
    std::vector<Position> getPath(const std::string& from_room, const std::string& to_room) const {
        std::string path_key = from_room + "_" + to_room;
        auto it = paths_.find(path_key);
        
        std::vector<Position> waypoints;
        if (it != paths_.end()) {
            for (const auto& wp : it->second) {
                waypoints.push_back({wp.x, wp.y, 0.0});  // Assuming yaw=0 for waypoints
            }
        }
        
        // Add final destination
        waypoints.push_back(getRoomPosition(to_room));
        return waypoints;
    }
private:
    // ADDED: Waypoint struct for 2D coordinates in paths
    struct Waypoint {
        double x;
        double y;
    };
    
    std::map<std::string, Position> rooms_;
    std::map<std::string, std::string> right_pattern_;
    std::map<std::string, std::string> left_pattern_;
    // ADDED: Declaration for paths_ (matches your assignments in constructor)
    std::map<std::string, std::vector<Waypoint>> paths_;
};
} // namespace real_maze_robots
#endif