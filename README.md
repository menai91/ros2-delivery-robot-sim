# ROS 2 Autonomous Mobile Robot (Gazebo Simulation)

A simulated differential-drive robot that maps an environment with SLAM, navigates it with Nav2, and runs missions through a C++ behavior-tree layer built on BehaviorTree.CPP. Built with ROS 2 Jazzy and Gazebo Harmonic. Simulation only.

## What it does

The robot runs in one of three modes:

1. **Mapping:** drive the robot around an unknown room with teleop while `slam_toolbox` builds a 2D map from lidar, then save the map to a file.
2. **Navigation:** load the saved map, click a goal in RViz, and Nav2 plans a path and drives the robot there.
3. **Missions:** `bt_runner` loads a behavior tree from an XML file and drives the robot through it. The current demo tree drives to 2 waypoints defined in the xml file.

## Architecture

| Package | Folder | Contents |
| --- | --- | --- |
| `ros2_project` | `src/robot_description/` | Robot model (URDF), Gazebo world, launch file, saved map, Nav2 and SLAM configuration |
| `amr_navigation` | `src/amr_navigation/` | `Navigator`: a non-blocking C++ wrapper around Nav2's `navigate_to_pose` action. No behavior-tree dependency. |
| `amr_autonomy` | `src/amr_autonomy/` | Behavior-tree nodes, node registration, mission trees (`trees/`), and the `bt_runner` executable |

Design notes:

- **Non-blocking navigation.** A behavior tree must return from every tick quickly, so `Navigator` exposes `start` / `get_status` / `cancel` instead of a blocking "drive to pose" call. The `MoveToPosition` tree node is a thin adapter that polls it each tick.
- **Stale-goal guard.** Cancelling an action is asynchronous, so callbacks for an old goal can still arrive, including a goal that Nav2 accepts after it was cancelled. Every callback carries a goal ID and ignores itself if a newer goal has started; a late accept for a cancelled goal is cancelled on arrival.
- **Goal-response timeout.** If Nav2 never answers a goal, `Navigator` fails it after a timeout instead of waiting forever. Limits on total drive time belong to the tree (`Timeout` decorator), where they are visible and set per task.
- **Clean shutdown.** On Ctrl-C, `bt_runner` halts the tree (cancelling the active goal), waits for outstanding replies from Nav2, and cancels any remaining goals before exiting, so the robot does not keep driving after the program ends.

## Built with

ROS 2 Jazzy, Gazebo Harmonic, Nav2, `slam_toolbox`, BehaviorTree.CPP 4, C++17.

## Building

```bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Launching

Everything starts from one launch file:

```bash
ros2 launch ros2_project gazebo_launch.py [world:=<file>] [use_slam:=<true|false>] [use_nav:=<true|false>]
```

| Argument | Default | Description |
| --- | --- | --- |
| `world` | `slam_world.sdf` | World file in `worlds/` to load. Nav2 looks for the map `maps/<world name>_map.yaml`, so `slam_world.sdf` uses `slam_world_map.yaml`. |
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

Click **2D Goal Pose** in RViz. Press on the destination, then drag to set the final heading.

## Running a mission

Start navigation mode first (see above), then in a second terminal:

```bash
source install/setup.bash
ros2 run amr_autonomy bt_runner $(ros2 pkg prefix --share amr_autonomy)/trees/navigate_demo.xml
```

Trees are plain XML and are loaded at run time, so a tree can be edited without recompiling. The command above uses the installed copy, so rebuild to install an edited tree, or pass the path to the file in `src/`. Ctrl-C stops the mission and the robot.

## Known limitations

- The robot's starting pose is fixed at the map origin in `nav2_params.yaml` (`initial_pose`), which matches the spawn point in the provided world (Nav2 needs an initial pose). A different spawn point, or a map made from a different start, needs that value changed, or the pose set manually with RViz's 2D Pose Estimate.
- The demo tree is a one-shot sequence: `bt_runner` exits when the tree finishes or fails.
- At startup, `bt_runner` waits up to 15 s for Nav2's action server to be discovered.

## Roadmap

- Failure policy as tree structure (retry, skip, timeout)
- Delivery mission: pickup and drop-off stations, task queue
- Simulated battery with automatic docking and charging
- Unit tests, launch tests and CI

## License

Apache 2.0, see `src/LICENSE`.
