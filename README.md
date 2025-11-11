# Fast-Planner ROS2

ROS2 port of the Fast-Planner path planning system.

## Overview

This is a ROS2 (Humble/Iron) port of the [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) path planning framework. Fast-Planner is a robust and efficient path planning system designed for quadrotor UAVs, featuring kinodynamic path searching and B-spline trajectory optimization.

## Features

- **Kinodynamic A* Search**: Efficient path searching considering dynamic constraints
- **B-spline Trajectory Optimization**: Smooth trajectory generation with obstacle avoidance
- **ESDF Map**: Euclidean Signed Distance Field for efficient collision checking
- **Reactive Replanning**: Dynamic replanning based on environment changes
- **ROS2 Native**: Fully ported to ROS2 with modern C++ and Python APIs

## Package Structure

- `bspline`: B-spline trajectory representation
- `bspline_opt`: B-spline trajectory optimization
- `path_searching`: Kinodynamic A* path search algorithm
- `plan_env`: Environment representation (ESDF, occupancy grid)
- `plan_manage`: Main planning node and FSM (Finite State Machine)
- `poly_traj`: Polynomial trajectory generation
- `traj_utils`: Trajectory utilities and visualization

## Dependencies

- ROS2 (Humble or Iron)
- Eigen3
- NLopt (nonlinear optimization library)
- PCL (Point Cloud Library)

## Installation

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/fast-planner-ros2.git
cd fast-planner-ros2

# Build with colcon
cd src
colcon build
source install/setup.bash
```

## Usage

```bash
# Launch the planning system
ros2 launch plan_manage kino_replan.launch.py
```

## Configuration

Edit `plan_manage/config/fast_planner_params.yaml` to configure planning parameters, waypoints, and optimization settings.

## License

This project maintains the same license as the original Fast-Planner project.

## Acknowledgments

Based on the original [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) by HKUST-Aerial-Robotics.



