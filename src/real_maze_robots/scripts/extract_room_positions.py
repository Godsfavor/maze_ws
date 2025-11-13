#!/usr/bin/env python3
"""
Interactive tool to extract room positions AND waypoints from a ROS map.
Click on room centers first, then add intermediate waypoints for paths.
"""

import cv2
import numpy as np
import yaml
import sys
from pathlib import Path

class MapCoordinateExtractor:
    def __init__(self, map_yaml_path):
        # Load map metadata
        with open(map_yaml_path, 'r') as f:
            self.map_info = yaml.safe_load(f)
        
        map_dir = Path(map_yaml_path).parent
        map_image_path = map_dir / self.map_info['image']
        
        # Load map image
        self.map_img = cv2.imread(str(map_image_path), cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise FileNotFoundError(f"Could not load map image: {map_image_path}")
        
        # Create color version for visualization
        self.original_img = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        
        # Scale factor for larger display
        self.scale_factor = 4.0
        
        # Resize display image
        self.display_img = cv2.resize(
            self.original_img,
            (int(self.original_img.shape[1] * self.scale_factor), 
             int(self.original_img.shape[0] * self.scale_factor)),
            interpolation=cv2.INTER_NEAREST
        )
        
        # Map parameters
        self.resolution = self.map_info['resolution']
        self.origin_x = self.map_info['origin'][0]
        self.origin_y = self.map_info['origin'][1]
        
        # Storage for points
        self.room_points = []  # First 4 points are rooms
        self.room_names = ['room_1', 'room_2', 'room_3', 'room_4']
        
        # Waypoint storage - organized by path
        self.current_path = None
        self.waypoints = {}  # Key: "room_X_room_Y", Value: list of (x,y) tuples
        
        # Mode: 'rooms' or 'waypoints'
        self.mode = 'rooms'
        
        # Path patterns for clockwise and counter-clockwise
        self.path_keys = {
            'clockwise': [
                'room_1_room_2', 'room_2_room_3', 
                'room_3_room_4', 'room_4_room_1'
            ],
            'counter_clockwise': [
                'room_1_room_4', 'room_4_room_3',
                'room_3_room_2', 'room_2_room_1'
            ]
        }
        
        print("=" * 70)
        print("📍 MAP COORDINATE EXTRACTOR (WITH WAYPOINTS)")
        print("=" * 70)
        print(f"Map: {map_image_path}")
        print(f"Resolution: {self.resolution} m/pixel")
        print(f"Origin: ({self.origin_x}, {self.origin_y})")
        print(f"Original map size: {self.map_img.shape[1]} x {self.map_img.shape[0]} pixels")
        print(f"Display scale: {self.scale_factor}x")
        print("=" * 70)
        self.print_instructions()
    
    def print_instructions(self):
        if self.mode == 'rooms':
            print("\n🏠 ROOM MODE - Click room centers in order:")
            print("  1. room_1 (lower-right)")
            print("  2. room_2 (lower-left)")
            print("  3. room_3 (upper-left)")
            print("  4. room_4 (upper-right)")
            print("\n  After 4 rooms, press 'w' to enter WAYPOINT MODE")
        else:
            print(f"\n🛤️  WAYPOINT MODE - Path: {self.current_path}")
            print("  Click on corridor points / doorways in order")
            print("  Press 'n' for NEXT path")
            print("  Press 'p' for PREVIOUS path (undo)")
        
        print("\n⌨️  KEYBOARD SHORTCUTS:")
        print("  'w' : Enter waypoint mode (after marking rooms)")
        print("  'n' : Next path (in waypoint mode)")
        print("  'p' : Previous path / undo last waypoint")
        print("  'r' : Reset everything")
        print("  's' : Save current progress")
        print("  'q' : Quit and save")
        print("  'd' : Draw lines between waypoints (toggle)")
        print("=" * 70)
    
    def pixel_to_world(self, px, py):
        """Convert pixel coordinates to world coordinates (meters)"""
        world_x = self.origin_x + (px * self.resolution)
        world_y = self.origin_y + ((self.map_img.shape[0] - py) * self.resolution)
        return world_x, world_y
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Convert scaled display coordinates to original pixel coordinates
            orig_x = int(x / self.scale_factor)
            orig_y = int(y / self.scale_factor)
            
            # Convert to world coordinates
            world_x, world_y = self.pixel_to_world(orig_x, orig_y)
            
            if self.mode == 'rooms':
                # Add room point
                if len(self.room_points) < 4:
                    room_name = self.room_names[len(self.room_points)]
                    self.room_points.append((world_x, world_y))
                    
                    # Draw on display
                    cv2.circle(self.display_img, (x, y), 8, (0, 255, 0), -1)
                    cv2.putText(self.display_img, room_name, (x + 15, y - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    print(f"✓ {room_name}: World({world_x:.2f}, {world_y:.2f})")
                    
                    if len(self.room_points) == 4:
                        print("\n🎉 All rooms marked! Press 'w' to add waypoints or 's' to save")
                else:
                    print("⚠️  All 4 rooms already marked. Press 'w' for waypoint mode")
            
            elif self.mode == 'waypoints':
                # Add waypoint to current path
                if self.current_path:
                    if self.current_path not in self.waypoints:
                        self.waypoints[self.current_path] = []
                    
                    self.waypoints[self.current_path].append((world_x, world_y))
                    
                    # Draw waypoint
                    cv2.circle(self.display_img, (x, y), 5, (255, 0, 0), -1)
                    wp_num = len(self.waypoints[self.current_path])
                    label = f"WP{wp_num}"
                    cv2.putText(self.display_img, label, (x + 10, y + 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                    
                    # Draw line from previous waypoint
                    if len(self.waypoints[self.current_path]) > 1:
                        prev_wx, prev_wy = self.waypoints[self.current_path][-2]
                        prev_px, prev_py = self.world_to_pixel(prev_wx, prev_wy)
                        cv2.line(self.display_img, 
                                (int(prev_px * self.scale_factor), int(prev_py * self.scale_factor)),
                                (x, y), (255, 0, 0), 2)
                    
                    print(f"  WP{wp_num}: World({world_x:.2f}, {world_y:.2f})")
    
    def world_to_pixel(self, world_x, world_y):
        """Convert world coordinates back to pixel coordinates"""
        px = (world_x - self.origin_x) / self.resolution
        py = self.map_img.shape[0] - ((world_y - self.origin_y) / self.resolution)
        return px, py
    
    def enter_waypoint_mode(self):
        """Switch to waypoint mode"""
        if len(self.room_points) < 4:
            print("⚠️  Need to mark all 4 rooms first!")
            return
        
        self.mode = 'waypoints'
        
        # Draw lines connecting rooms in clockwise order
        self.redraw_rooms()
        
        # Start with first clockwise path
        self.current_path_index = 0
        self.current_direction = 'clockwise'
        self.current_path = self.path_keys[self.current_direction][self.current_path_index]
        
        print(f"\n🛤️  WAYPOINT MODE")
        print(f"Path: {self.current_path}")
        print("Click waypoints along the safe corridor from room to room")
        self.print_path_info()
    
    def redraw_rooms(self):
        """Redraw room markers on display"""
        for i, (world_x, world_y) in enumerate(self.room_points):
            px, py = self.world_to_pixel(world_x, world_y)
            disp_x = int(px * self.scale_factor)
            disp_y = int(py * self.scale_factor)
            
            cv2.circle(self.display_img, (disp_x, disp_y), 8, (0, 255, 0), -1)
            cv2.putText(self.display_img, self.room_names[i], 
                       (disp_x + 15, disp_y - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    def print_path_info(self):
        """Show current path being edited"""
        parts = self.current_path.split('_')
        from_room = f"{parts[0]}_{parts[1]}"
        to_room = f"{parts[2]}_{parts[3]}"
        
        print(f"\n📍 Current: {from_room} → {to_room}")
        print(f"   Direction: {self.current_direction}")
        
        if self.current_path in self.waypoints:
            print(f"   Waypoints added: {len(self.waypoints[self.current_path])}")
    
    def next_path(self):
        """Move to next path"""
        if self.mode != 'waypoints':
            return
        
        # Save current and move to next
        self.current_path_index += 1
        
        # Check if we finished clockwise, switch to counter-clockwise
        if self.current_path_index >= len(self.path_keys[self.current_direction]):
            if self.current_direction == 'clockwise':
                self.current_direction = 'counter_clockwise'
                self.current_path_index = 0
                print("\n🔄 Switching to COUNTER-CLOCKWISE paths")
            else:
                print("\n✅ All paths completed!")
                self.save_points()
                return
        
        self.current_path = self.path_keys[self.current_direction][self.current_path_index]
        self.redraw_with_all_waypoints()
        self.print_path_info()
    
    def prev_path(self):
        """Undo last waypoint or go to previous path"""
        if self.mode != 'waypoints':
            return
        
        # Remove last waypoint from current path
        if self.current_path in self.waypoints and len(self.waypoints[self.current_path]) > 0:
            self.waypoints[self.current_path].pop()
            print(f"⬅️  Removed last waypoint from {self.current_path}")
            self.redraw_with_all_waypoints()
    
    def redraw_with_all_waypoints(self):
        """Redraw display with all saved waypoints"""
        # Reset display
        self.display_img = cv2.resize(
            self.original_img,
            (int(self.original_img.shape[1] * self.scale_factor),
             int(self.original_img.shape[0] * self.scale_factor)),
            interpolation=cv2.INTER_NEAREST
        )
        
        # Redraw rooms
        self.redraw_rooms()
        
        # Redraw all waypoints
        for path_key, waypoint_list in self.waypoints.items():
            color = (255, 0, 0) if path_key == self.current_path else (150, 150, 150)
            
            for i, (wx, wy) in enumerate(waypoint_list):
                px, py = self.world_to_pixel(wx, wy)
                disp_x = int(px * self.scale_factor)
                disp_y = int(py * self.scale_factor)
                
                cv2.circle(self.display_img, (disp_x, disp_y), 5, color, -1)
                
                if i > 0:
                    prev_wx, prev_wy = waypoint_list[i-1]
                    prev_px, prev_py = self.world_to_pixel(prev_wx, prev_wy)
                    cv2.line(self.display_img,
                            (int(prev_px * self.scale_factor), int(prev_py * self.scale_factor)),
                            (disp_x, disp_y), color, 2)
    
    def reset_points(self):
        """Clear all points and reset display"""
        self.room_points = []
        self.waypoints = {}
        self.mode = 'rooms'
        self.current_path = None
        
        self.display_img = cv2.resize(
            self.original_img,
            (int(self.original_img.shape[1] * self.scale_factor),
             int(self.original_img.shape[0] * self.scale_factor)),
            interpolation=cv2.INTER_NEAREST
        )
        print("\n🔄 Reset all points")
        self.print_instructions()
    
    def save_points(self, output_file='room_positions.yaml'):
        """Save extracted coordinates to YAML file"""
        if len(self.room_points) < 4:
            print(f"\n⚠️  Need at least 4 room positions (have {len(self.room_points)})")
            return
        
        # Prepare output data
        output_data = {
            'map_info': {
                'resolution': self.resolution,
                'origin': [self.origin_x, self.origin_y, 0]
            },
            'rooms': {}
        }
        
        # Add rooms
        yaws = [1.57, 3.14, 0.0, -1.57]  # room_1, room_2, room_3, room_4
        for i, (name, (x, y)) in enumerate(zip(self.room_names, self.room_points)):
            output_data['rooms'][name] = {
                'x': float(x),
                'y': float(y),
                'yaw': yaws[i]
            }
        
        # Add waypoints
        if self.waypoints:
            output_data['paths'] = {}
            for path_key, waypoint_list in self.waypoints.items():
                output_data['paths'][path_key] = [
                    {'x': float(wx), 'y': float(wy)} 
                    for wx, wy in waypoint_list
                ]
        
        # Save to file
        with open(output_file, 'w') as f:
            yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)
        
        print(f"\n✅ Saved to {output_file}")
        self._print_cpp_code()
    
    def _print_cpp_code(self):
        """Print C++ code snippet for room_manager.hpp"""
        print("\n" + "=" * 70)
        print("📋 COPY THIS INTO room_manager.hpp:")
        print("=" * 70)
        
        # Room positions
        print("\n// Room positions (measured from map)")
        yaws = [1.57, 3.14, 0.0, -1.57]
        for i, (name, (x, y)) in enumerate(zip(self.room_names, self.room_points)):
            print(f'rooms_["{name}"] = {{{x:.2f}, {y:.2f}, {yaws[i]:.2f}}};')
        
        # Waypoints
        if self.waypoints:
            print("\n// Waypoints for each path")
            for path_key in self.path_keys['clockwise'] + self.path_keys['counter_clockwise']:
                if path_key in self.waypoints and len(self.waypoints[path_key]) > 0:
                    waypoints_str = ", ".join([f"{{{wx:.2f}, {wy:.2f}}}" 
                                              for wx, wy in self.waypoints[path_key]])
                    print(f'paths_["{path_key}"] = {{ {waypoints_str} }};')
                else:
                    print(f'// paths_["{path_key}"] = {{ }}; // No waypoints needed')
        
        print("=" * 70)
    
    def run(self):
        """Main interactive loop"""
        cv2.namedWindow('Map - Click Room Centers', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Map - Click Room Centers', self.mouse_callback)
        
        while True:
            cv2.imshow('Map - Click Room Centers', self.display_img)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n👋 Quitting...")
                if len(self.room_points) >= 4:
                    self.save_points()
                break
            elif key == ord('r'):
                self.reset_points()
            elif key == ord('s'):
                self.save_points()
            elif key == ord('w'):
                self.enter_waypoint_mode()
            elif key == ord('n'):
                self.next_path()
            elif key == ord('p'):
                self.prev_path()
        
        cv2.destroyAllWindows()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 extract_room_positions.py <path_to_map.yaml>")
        print("Example: python3 extract_room_positions.py ~/maze_ws/src/mazesim/maps/my_map_1.yaml")
        sys.exit(1)
    
    try:
        extractor = MapCoordinateExtractor(sys.argv[1])
        extractor.run()
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)