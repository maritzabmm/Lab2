# Task 3 - TF2 Transforms Complete Guide

## Overview
This package implements Task 3 requirements for TF2 transforms in ROS2. It includes:

1. **tf_broadcaster.py** - Creates a robot frame moving in circular path around map origin
2. **static_lidar_broadcaster.py** - Static transform from robot to lidar frame  
3. **dynamic_scanner_broadcaster.py** - Dynamic scanner frame oscillating left-right
4. **transform_monitor.py** - Listens to transforms, calculates distance and speed
5. **revolution_counter.py** - Counts robot revolutions around origin

## Building the Package

Navigate to your workspace root and build:
```bash
cd ~/ros2_lab2_ws
colcon build --packages-select task3
source install/setup.bash
```

## Running the Nodes

### Terminal 1: Base Robot Movement
Start the main robot broadcaster that creates the circular movement:
```bash
ros2 run task3 tf_broadcaster
```

### Terminal 2: Static Lidar Transform
Start the static lidar transform broadcaster:
```bash
ros2 run task3 static_lidar_broadcaster  
```

### Terminal 3: Dynamic Scanner Transform
Start the oscillating scanner broadcaster:
```bash
ros2 run task3 dynamic_scanner_broadcaster
```

### Terminal 4: Transform Monitor
Monitor transforms and calculate speed:
```bash
ros2 run task3 transform_monitor
```

### Terminal 5: Revolution Counter
Count robot revolutions:
```bash
ros2 run task3 revolution_counter
```

### Terminal 6: RViz2 Visualization
Visualize all transforms:
```bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set Fixed Frame to "map"
2. Add -> By Topic -> TF -> OK
3. You should see all the coordinate frames

### Terminal 7: TF Tree Visualization
View the transform tree:
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

## Expected Results

### Transform Hierarchy
```
map
└── robot (circular motion)
    ├── lidar (static: +0.2x, +0.3z)
    └── scanner (dynamic: oscillating ±0.3y, +0.1x, +0.4z)
```

### Console Output Examples

**transform_monitor** will output every 2 seconds:
```
[INFO] Frame: lidar | Position: (1.000, 0.000, 0.300) | Distance from origin: 1.044m | Speed: 1.571m/s
```

**revolution_counter** will output:
```
[INFO] Position: (1.000, 0.000) | Angle: 0.0° | Total revolutions: 0.159 | Complete revolutions: 0
```

## Key Features Implemented

1. **Static Transform**: Lidar frame positioned 20cm forward and 30cm up from robot
2. **Dynamic Transform**: Scanner oscillates ±30cm left-right with 1Hz frequency
3. **Distance Monitoring**: Calculates distance from map origin every 2 seconds
4. **Speed Calculation**: Computes frame velocity relative to map
5. **Revolution Counting**: Tracks complete revolutions using angle accumulation

## Verification Checklist

- [ ] All 5 nodes running without errors
- [ ] RViz shows map, robot, lidar, and scanner frames
- [ ] Robot frame moves in circular path
- [ ] Lidar frame follows robot (static offset)
- [ ] Scanner frame oscillates left-right while following robot
- [ ] Transform monitor prints distance and speed
- [ ] Revolution counter tracks revolutions
- [ ] TF tree shows correct hierarchy

## Troubleshooting

If you encounter import errors, ensure:
1. ROS2 environment is sourced: `source /opt/ros/humble/setup.bash`
2. Workspace is built and sourced: `source ~/ros2_lab2_ws/install/setup.bash`
3. All dependencies are installed: `sudo apt install ros-humble-tf-transformations`

## Commands Summary

```bash
# Build
cd ~/ros2_lab2_ws && colcon build --packages-select task3 && source install/setup.bash

# Run all nodes (in separate terminals)
ros2 run task3 tf_broadcaster
ros2 run task3 static_lidar_broadcaster  
ros2 run task3 dynamic_scanner_broadcaster
ros2 run task3 transform_monitor
ros2 run task3 revolution_counter

# Visualization
ros2 run rviz2 rviz2
ros2 run rqt_tf_tree rqt_tf_tree

# Optional: Check specific transforms
ros2 run tf2_ros tf2_echo map robot
ros2 run tf2_ros tf2_echo robot lidar
ros2 run tf2_ros tf2_echo robot scanner
```