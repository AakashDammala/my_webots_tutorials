# Walker Robot - Webots ROS2 Simulation

A ROS2 package implementing a simple walker algorithm for the ePuck robot in Webots. The robot navigates autonomously by moving forward until obstacles are detected, then rotating (alternating clockwise/counterclockwise) until the path clears.

## Features

- **State Machine Architecture**: Uses the State design pattern for clean walker behavior management
- **Obstacle Avoidance**: Detects obstacles via proximity sensors and rotates to find clear paths
- **Rosbag Recording**: Captures all topics except `/camera/*` to minimize file size
- **OOP Design**: Testable class-based implementation supporting potential unit testing

## Dependencies

- ROS 2 Humble
- Webots 2025a
- webots_ros2_epuck (simulator integration)

## Build

```bash
colcon build --packages-select my_webots_tutorials
source install/setup.bash
```

## Running the Simulation

### Launch walker with Webots
```bash
ros2 launch my_webots_tutorials walker_launch.py
```

### Launch with RViz visualization
```bash
ros2 launch my_webots_tutorials walker_launch.py rviz:=true
```

### Launch with rosbag recording (all topics except /camera/*)
```bash
ros2 launch my_webots_tutorials walker_launch.py rosbag:=true
```

### Launch with both RViz and rosbag
```bash
ros2 launch my_webots_tutorials walker_launch.py rviz:=true rosbag:=true
```

Rosbag files are saved to the `results/` directory (created relative to launch execution).

## Rosbag Operations

### Inspect recorded bag file
```bash
ros2 bag info results/rosbag2_*
```

### Play back bag file (Webots should not be running)
```bash
ros2 bag play results/rosbag2_*
```

### Compress bag (reduces file size)
```bash
ros2 bag compress results/rosbag2_*
```

## Project Structure

- `src/walker.cpp` - Walker ROS2 node with state machine implementation
- `include/my_webots_tutorials/walker.hpp` - Walker class definition
- `launch/walker_launch.py` - Launch file with rosbag recording control
- `results/` - Directory for recorded bag files

## Design Notes

- **State Pattern**: Walker implements a finite state machine (Moving, Rotating states)
- **Doxygen Comments**: All C++ files include Doxygen-compatible documentation
- **Google C++ Style Guide**: Code follows Google style conventions
- **Testability**: Classes designed to support future unit testing

## License

See LICENSE file in repository.

