#!/usr/bin/env python3

import numpy as np
from PIL import Image
import yaml
import sys
import os

def load_map(pgm_file, yaml_file):
    """Load PGM map and its metadata"""
    # Load image
    img = Image.open(pgm_file)
    map_data = np.array(img)
    
    # Load metadata
    with open(yaml_file, 'r') as f:
        metadata = yaml.safe_load(f)
    
    return map_data, metadata

def find_walls(map_data, threshold=250):
    """Find wall pixels (dark pixels) in the map"""
    walls = map_data < threshold  # Occupied cells
    return walls

def pixel_to_world(pixel_x, pixel_y, metadata):
    """Convert pixel coordinates to world coordinates"""
    resolution = metadata['resolution']
    origin = metadata['origin']
    
    # Pixel (0,0) is top-left, but map origin is bottom-left
    # So we need to flip y
    height = map_data.shape[0]
    
    world_x = origin[0] + pixel_x * resolution
    world_y = origin[1] + (height - pixel_y - 1) * resolution
    
    return world_x, world_y

def create_wall_boxes(walls, metadata, min_wall_length=5):
    """Create rectangular wall segments from occupied pixels"""
    wall_boxes = []
    
    # Simple approach: find horizontal and vertical wall segments
    height, width = walls.shape
    visited = np.zeros_like(walls, dtype=bool)
    
    # Find horizontal walls
    for y in range(height):
        x = 0
        while x < width:
            if walls[y, x] and not visited[y, x]:
                # Start of a wall segment
                start_x = x
                while x < width and walls[y, x]:
                    visited[y, x] = True
                    x += 1
                end_x = x - 1
                
                if end_x - start_x + 1 >= min_wall_length:
                    # Convert to world coordinates
                    center_x_pixel = (start_x + end_x) / 2.0
                    center_y_pixel = y
                    length = (end_x - start_x + 1) * metadata['resolution']
                    width_wall = metadata['resolution']
                    
                    center_x, center_y = pixel_to_world(center_x_pixel, center_y_pixel, metadata)
                    
                    wall_boxes.append({
                        'x': center_x,
                        'y': center_y,
                        'z': 1.0,  # Height of wall
                        'length': length,
                        'width': width_wall,
                        'height': 2.0,
                        'orientation': 'horizontal'
                    })
            else:
                x += 1
    
    # Find vertical walls
    visited = np.zeros_like(walls, dtype=bool)
    for x in range(width):
        y = 0
        while y < height:
            if walls[y, x] and not visited[y, x]:
                # Start of a wall segment
                start_y = y
                while y < height and walls[y, x]:
                    visited[y, x] = True
                    y += 1
                end_y = y - 1
                
                if end_y - start_y + 1 >= min_wall_length:
                    # Convert to world coordinates
                    center_x_pixel = x
                    center_y_pixel = (start_y + end_y) / 2.0
                    width_wall = metadata['resolution']
                    length = (end_y - start_y + 1) * metadata['resolution']
                    
                    center_x, center_y = pixel_to_world(center_x_pixel, center_y_pixel, metadata)
                    
                    wall_boxes.append({
                        'x': center_x,
                        'y': center_y,
                        'z': 1.0,
                        'length': width_wall,
                        'width': length,
                        'height': 2.0,
                        'orientation': 'vertical'
                    })
            else:
                y += 1
    
    return wall_boxes

def generate_sdf(wall_boxes, output_file):
    """Generate Gazebo SDF file from wall boxes"""
    
    sdf_header = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="maze_map">
    <static>true</static>
    
'''
    
    sdf_footer = '''  </model>
</sdf>
'''
    
    with open(output_file, 'w') as f:
        f.write(sdf_header)
        
        # Write each wall
        for i, wall in enumerate(wall_boxes):
            wall_sdf = f'''    <link name="wall_{i}">
      <pose>{wall['x']} {wall['y']} {wall['z']} 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>{wall['length']} {wall['width']} {wall['height']}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{wall['length']} {wall['width']} {wall['height']}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>
    
'''
            f.write(wall_sdf)
        
        f.write(sdf_footer)
    
    print(f"Generated SDF with {len(wall_boxes)} walls")

if __name__ == "__main__":
    # Get package path
    mazesim_dir = os.path.expanduser("~/maze_ws/src/mazesim")
    
    pgm_file = os.path.join(mazesim_dir, "maps", "my_map_1.pgm")
    yaml_file = os.path.join(mazesim_dir, "maps", "my_map_1.yaml")
    output_file = os.path.join(mazesim_dir, "models", "maze_map", "model.sdf")
    
    print(f"Loading map from {pgm_file}")
    map_data, metadata = load_map(pgm_file, yaml_file)
    
    print(f"Map size: {map_data.shape}")
    print(f"Resolution: {metadata['resolution']} m/pixel")
    print(f"Origin: {metadata['origin']}")
    
    print("Finding walls...")
    walls = find_walls(map_data)
    print(f"Wall pixels: {np.sum(walls)}")
    
    print("Creating wall boxes...")
    wall_boxes = create_wall_boxes(walls, metadata, min_wall_length=3)
    
    print(f"Generating SDF to {output_file}")
    generate_sdf(wall_boxes, output_file)
    
    print("Done!")
