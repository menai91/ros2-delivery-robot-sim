# ROS 2 Autonomous Mobile Robot (Gazebo Simulation)

A simulated robot that maps an environment with SLAM and then navigates it autonomously with Nav2, built with ROS 2 Jazzy and Gazebo Harmonic.

Simulation only (Gazebo Harmonic).

## What it does

The robot runs in one of two modes:

1. **Mapping:** drive the robot around an unknown room with teleop while `slam_toolbox` builds a 2D map from lidar, then save the map to a file.
2. **Navigation:** load the saved map, click a goal in RViz, and Nav2 plans a path and drives the robot there.

## Built with

`slam_toolbox`, Nav2, ROS 2 Jazzy, Gazebo Harmonic.
## Launching

Everything starts from one launch file:

```bash
ros2 launch ros2_project gazebo_launch.py [world:=<file>] [use_slam:=<true|false>] [use_nav:=<true|false>]
```

| Argument | Default | Description |
| --- | --- | --- |
| `world` | `my_world.sdf` | World file in `worlds/` to load. Nav2 looks for the map `maps/<world name>_map.yaml`, so `slam_world.sdf` uses `slam_world_map.yaml`. |
| `use_slam` | `false` | Start `slam_toolbox` to build a map (mapping mode). |
| `use_nav` | `false` | Start Nav2 with the saved map for the chosen world (navigation mode). |

Gazebo, the ROS-Gazebo bridge, `robot_state_publisher` and RViz always start.

**Never set `use_slam` and `use_nav` to `true` together.** Both publish the `map -> odom` transform and the `/map` topic, and they fight over them.

### Mapping mode

```bash
ros2 launch ros2_project gazebo_launch.py world:=slam_world.sdf use_slam:=true
```

Drive the robot around with teleop, then save the map into `maps/` as `<world name>_map` (for example `slam_world_map`). Rebuild the package afterwards so the new files are installed.

### Navigation mode

```bash
ros2 launch ros2_project gazebo_launch.py world:=slam_world.sdf use_nav:=true
```

Click **2D Goal Pose** in RViz. Press on the destination, then drag to set the final heading

## Known limitations

- The robot's starting pose is fixed at the map origin in `nav2_params.yaml` (`initial_pose`), which matches the spawn point in the provided world (This is due to the fact Nav2 needs a pose). A different spawn point or a map made from a different start needs that value changed, or the pose set manually with RViz's 2D Pose Estimate.
  
