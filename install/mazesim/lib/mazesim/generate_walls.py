#!/usr/bin/env python3
"""
Generate Gazebo wall models from occupancy grid map
"""

import numpy as np
from PIL import Image
import yaml
import sys

def load_map(pgm_file, yaml_file):
    """Load map from PGM and YAML files"""
    # Load image
    img = Image.open(pgm_file)
    map_data = np.array(img)
    
    # Load metadata
    with open(yaml_file, 'r') as f:
        metadata = yaml.safe_load(f)
    
    return map_data, metadata

def find_walls(map_data, threshold=250):
    """
    Find wall pixels (black pixels)
    Returns list of wall coordinates in pixel space
    """
    # In PGM: 0 = black (occupied), 255 = white (free), 205 = gray (unknown)
    walls = np.argwhere(map_data < threshold)
    return walls

def pixel_to_world(pixel_x, pixel_y, origin, resolution):
    """Convert pixel coordinates to world coordinates"""
    world_x = origin[0] + (pixel_x * resolution)
    world_y = origin[1] + (pixel_y * resolution)
    return world_x, world_y

def generate_wall_sdf(walls, origin, resolution, output_file):
    """Generate SDF file with wall boxes"""
    
    print(f"Generating walls from {len(walls)} occupied pixels...")
    
    # Sample walls (use every Nth pixel to reduce complexity)
    sample_rate = 3  # Use every 3rd pixel
    sampled_walls = walls[::sample_rate]
    
    print(f"Sampled down to {len(sampled_walls)} wall segments")
    
    sdf_content = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="maze_walls">
    <static>true</static>
    <link name="walls">
'''
    
    wall_height = 2.5  # meters
    wall_thickness = 0.05  # Use resolution as thickness
    
    for idx, (py, px) in enumerate(sampled_walls):
        # Convert to world coordinates
        wx, wy = pixel_to_world(px, py, origin, resolution)
        
        # Create a small box for each wall pixel
        sdf_content += f'''
      <collision name="wall_{idx}">
        <pose>{wx} {wy} {wall_height/2} 0 0 0</pose>
        <geometry>
          <box>
            <size>{resolution} {resolution} {wall_height}</size>
          </box>
        </geometry>
      </collision>
      
      <visual name="wall_{idx}">
        <pose>{wx} {wy} {wall_height/2} 0 0 0</pose>
        <geometry>
          <box>
            <size>{resolution} {resolution} {wall_height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.01 0.01 0.01 1</specular>
        </material>
      </visual>
'''
    
    sdf_content += '''
    </link>
  </model>
</sdf>
'''
    
    with open(output_file, 'w') as f:
        f.write(sdf_content)
    
    print(f"SDF file written to {output_file}")

def main():
    # Paths
    map_dir = "../maps"
    pgm_file = f"{map_dir}/my_map_1.pgm"
    yaml_file = f"{map_dir}/my_map_1.yaml"
    output_file = "../models/maze_walls/model.sdf"
    
    print("Loading map...")
    map_data, metadata = load_map(pgm_file, yaml_file)
    
    print(f"Map size: {map_data.shape}")
    print(f"Resolution: {metadata['resolution']} m/pixel")
    print(f"Origin: {metadata['origin']}")
    
    print("Finding walls...")
    walls = find_walls(map_data)
    
    print("Generating SDF...")
    generate_wall_sdf(walls, metadata['origin'], metadata['resolution'], output_file)
    
    print("Done!")

if __name__ == "__main__":
    main()
