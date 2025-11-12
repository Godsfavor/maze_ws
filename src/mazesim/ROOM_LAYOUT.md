# Room Layout and Rotation Patterns

## Map Overview

- **Resolution**: 0.05 m/pixel (5 cm per pixel)
- **Origin**: (-2.44, -2.65, 0) meters at bottom-left corner
- **Dimensions**: Approximately 25.6 × 25.6 meters

## Room Positions

### Room 0: Upper-Left Room
- **Position**: (4.06, 3.85, 0.0) meters
- **Orientation**: 0.0 radians (facing right/east)
- **Description**: Large open space in upper-left quadrant

### Room 1: Upper-Right Room
- **Position**: (15.56, 6.35, 0.0) meters
- **Orientation**: -1.57 radians (facing down/south)
- **Description**: Open area in upper-right quadrant

### Room 2: Lower-Left Room
- **Position**: (4.06, 14.85, 0.0) meters
- **Orientation**: 1.57 radians (facing up/north)
- **Description**: Space in lower-left quadrant

### Room 3: Lower-Right Room
- **Position**: (15.56, 14.85, 0.0) meters
- **Orientation**: 3.14 radians (facing left/west)
- **Description**: Area in lower-right quadrant

## Rotation Patterns

### Clockwise Rotation (Right Signal: `/right`)
```
Room 0 → Room 1 → Room 3 → Room 2 → Room 0 (cycle repeats)
```

Visual representation:
```
┌─────────┐      ┌─────────┐
│ Room 0  │ ───► │ Room 1  │
│         │      │         │
└─────────┘      └────┬────┘
     ▲                │
     │                ▼
┌────┴────┐      ┌─────────┐
│ Room 2  │ ◄─── │ Room 3  │
│         │      │         │
└─────────┘      └─────────┘
```

### Counter-Clockwise Rotation (Left Signal: `/left`)
```
Room 0 → Room 2 → Room 3 → Room 1 → Room 0 (cycle repeats)
```

Visual representation:
```
┌─────────┐      ┌─────────┐
│ Room 0  │ ◄─── │ Room 1  │
│         │      │         │
└────┬────┘      └─────────┘
     │                ▲
     ▼                │
┌─────────┐      ┌────┴────┐
│ Room 2  │ ───► │ Room 3  │
│         │      │         │
└─────────┘      └─────────┘
```

## Initial Robot Assignments

- **Robot 1**: Room 0 (Upper-Left)
- **Robot 2**: Room 1 (Upper-Right)
- **Robot 3**: Room 2 (Lower-Left)
- **Robot 4**: Room 3 (Lower-Right)

## Movement Parameters

- **Linear Speed**: 0.3 m/s
- **Angular Speed**: 0.5 rad/s
- **Position Tolerance**: 0.15 m (arrives within 15 cm of target)
- **Angle Tolerance**: 0.1 rad (≈ 5.7 degrees)



cd ~/maze_ws
colcon build --symlink-install
source install/setup.bash


source /usr/share/gazebo/setup.bash
ros2 launch mazesim complete_simulation_with_odom_fix.launch.py


source ~/maze_ws/install/setup.bash

# Send LEFT command - all robots will rotate counter-clockwise
ros2 topic pub /left example_interfaces/msg/Empty "{}" --once

# Wait for navigation to complete, then send RIGHT command
ros2 topic pub /right example_interfaces/msg/Empty "{}" --once


ros2 run rqt_tf_tree rqt_tf_tree --force-discover

