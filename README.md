# ROS 2 Autonomous Mobile Robot (Gazebo Simulation)

A simulated differential-drive robot that maps an environment with SLAM, navigates it with Nav2, and runs missions through a C++ behavior-tree layer built on BehaviorTree.CPP. Built with ROS 2 Jazzy and Gazebo Harmonic. Simulation only.

https://github.com/user-attachments/assets/6ad3dcd4-9123-432f-8862-518ca15df85e

## What it does

The robot runs in one of three modes:

1. **Mapping:** drive the robot around an unknown room with teleop while `slam_toolbox` builds a 2D map from lidar, then save the map to a file.
2. **Navigation:** load the saved map, click a goal in RViz, and Nav2 plans a path and drives the robot there.
3. **Missions:** `bt_runner` loads a behavior tree from an XML file and drives the robot through it. Two trees are included:
   - `single_delivery.xml` runs one delivery: drive to a pickup station, load, drive to a dropoff station, unload. Then it exits.
   - `delivery_queue.xml` keeps running and works through delivery requests sent to the `/delivery_request` service, one after another. When there is nothing to do, it waits and checks again every second.

   Loading and unloading are stubs for now: the robot waits 5 s at each station.

## Architecture

| Package | Folder | Contents |
| --- | --- | --- |
| `ros2_project` | `src/robot_description/` | Robot model (URDF, written as xacro), Gazebo world, launch file, saved map, Nav2 and SLAM configuration |
| `amr_navigation` | `src/amr_navigation/` | `Navigator`: a non-blocking C++ wrapper around Nav2's `navigate_to_pose` action. A YAML loader for named station poses. No behavior-tree dependency. |
| `amr_interfaces` | `src/amr_interfaces/` | The `DeliveryRequest` service and the `Location` message it uses |
| `amr_autonomy` | `src/amr_autonomy/` | Behavior-tree nodes (`MoveToPosition`, `GetNextTask`, plus `PickupItem` and `DropoffItem` stubs), node registration, mission trees (`trees/`), the task queue and the `/delivery_request` server, station poses (`config/stations.yaml`), a launch file, and the `bt_runner` executable |

Design notes:

- **Non-blocking navigation.** A behavior tree must return from every tick quickly, so `Navigator` exposes `start` / `get_status` / `cancel` instead of a blocking "drive to pose" call. The `MoveToPosition` tree node is a thin adapter that polls it each tick.
- **Stale-goal guard.** Cancelling an action is asynchronous, so callbacks for an old goal can still arrive, including a goal that Nav2 accepts after it was cancelled. Every callback carries a goal ID and ignores itself if a newer goal has started; a late accept for a cancelled goal is cancelled on arrival.
- **Goal-response timeout.** If Nav2 never answers a goal, `Navigator` fails it after a timeout instead of waiting forever. Limits on total drive time belong to the tree (`Timeout` decorator), where they are visible and set per task.
- **Failure policy as tree structure.** Retries, timeouts and when to give up are written in the XML, not in C++, so they are visible and can be set per task. Each move gets up to 3 attempts of 60 s each. `MoveToPosition` writes Nav2's error code to the blackboard. Errors that retrying cannot fix (204, goal outside the map; 208, no valid path) end the retries at once. If a move still fails, the delivery fails instead of continuing to the dropoff without the item. Nav2 still runs its own recovery behaviors inside each attempt; the tree only makes mission-level choices.
- **Delivery requests come in through a ROS service.** A service, unlike a topic, lets `bt_runner` reply whether it accepted the request. A request can give each location as a station name from `stations.yaml` or as a pose, given as numbers (x, y and yaw). Names are checked when the request arrives, so an unknown name is rejected, and the tree only ever sees poses. Accepted tasks go into a plain C++ queue, and a `GetNextTask` tree node takes the next one when the robot is free. Charger poses are kept in a separate section of the file, so they can't be used as delivery stations.
- **Clean shutdown.** On Ctrl-C, `bt_runner` halts the tree (cancelling the active goal), waits for outstanding replies from Nav2, and cancels any remaining goals before exiting, so the robot does not keep driving after the program ends.

## Built with

ROS 2 Jazzy, Gazebo Harmonic, Nav2, `slam_toolbox`, BehaviorTree.CPP 4, yaml-cpp, C++17.

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
ros2 run amr_autonomy bt_runner single_delivery
```

`bt_runner` takes either the name of an installed tree (`single_delivery`, `delivery_queue`) or a path to a tree XML file. Trees are plain XML and are loaded at run time, so a tree can be edited without recompiling. A name uses the installed copy, so rebuild to install an edited tree, or pass the path to the file in `src/` (for example `src/amr_autonomy/trees/single_delivery.xml`). Ctrl-C stops the mission and the robot.

The same can be started with a launch file, which runs `delivery_queue` unless another tree is given:

```bash
ros2 launch amr_autonomy bt_runner_launch.py [tree:=<name or path>]
```

### Delivery requests

To run deliveries on request, start the queue tree:

```bash
ros2 run amr_autonomy bt_runner delivery_queue
```

Then send requests from another terminal. By station name:

```bash
ros2 service call /delivery_request amr_interfaces/srv/DeliveryRequest \
  "{pickup: {station: room1_station}, dropoff: {station: room2_station}}"
```

Or with a raw pose (x, y in metres, yaw in radians, map frame) for either location. Leave `station` out to use the pose:

```bash
ros2 service call /delivery_request amr_interfaces/srv/DeliveryRequest \
  "{pickup: {x: 5.5, y: -3.5, yaw: 3.14}, dropoff: {station: room2_station}}"
```

The reply says whether the request was accepted, and why not if it was rejected. Station poses are read from the installed `config/stations.yaml`, so rebuild after editing it.

### Example output

<details>
<summary>Two queued deliveries, plus a rejected request</summary>

Requesting deliveries from a second terminal:

```text
$ ros2 service call /delivery_request amr_interfaces/srv/DeliveryRequest \
    "{pickup: {station: room1_station}, dropoff: {station: room2_station}}"
response: accepted=True, message='task has been queued'

$ ros2 service call /delivery_request amr_interfaces/srv/DeliveryRequest \
    "{pickup: {station: room1_station}, dropoff: {station: room22_station}}"
response: accepted=False, message="unknown dropoff station 'room22_station'"
```

Robot log. The second request arrives while goal #1 is still driving, and the robot starts it as soon as the first delivery finishes:

```text
[bt_runner]: Connected to 'navigate_to_pose' action server
[bt_runner]: Accepting delivery requests on '/delivery_request' (2 stations loaded)
[bt_runner]: Loaded behavior tree from '.../trees/delivery_queue.xml'
[bt_runner]: Queued delivery: pickup (5.50, -3.50, 3.14) -> dropoff (1.50, -3.50, 3.14), 1 task(s) waiting
[bt_runner]: Sending goal #1: x=5.50, y=-3.50, yaw=3.14
[bt_runner]: Goal #1 accepted, navigating
[bt_runner]: Queued delivery: pickup (5.50, -3.50, 3.14) -> dropoff (1.50, -3.50, 3.14), 1 task(s) waiting
[bt_runner]: Goal #1 reached
[PickupItem]: [PickupItem] Started
[PickupItem]: [PickupItem] Finished
[bt_runner]: Sending goal #2: x=1.50, y=-3.50, yaw=3.14
[bt_runner]: Goal #2 accepted, navigating
[bt_runner]: Goal #2 reached
[DropoffItem]: [DropoffItem] Started
[DropoffItem]: [DropoffItem] Finished
[bt_runner]: Sending goal #3: x=5.50, y=-3.50, yaw=3.14
... second delivery repeats the same sequence (goals #3 and #4) ...
[DropoffItem]: [DropoffItem] Finished
```

</details>

## Known limitations

- The robot's starting pose is fixed at the map origin in `nav2_params.yaml` (`initial_pose`), which matches the spawn point in the provided world (Nav2 needs an initial pose). A different spawn point, or a map made from a different start, needs that value changed, or the pose set manually with RViz's 2D Pose Estimate.
- If a delivery fails (for example, a station can't be reached), the queue tree drops that task and moves on to the next one.
- The queue only exists in memory. Tasks that are still waiting are lost when `bt_runner` stops.
- When a goal can't be reached, the tree does not find out right away. Nav2 can't tell a wall from an obstacle that might move, so it first spends about 17 s trying to clear the way: spinning, backing up and checking again. Only after that does it report the goal as unreachable.
- At startup, `bt_runner` waits up to 15 s for Nav2's action server to be discovered.
- The wheel collision cylinders are rotated by 1.5708 rad (the `quarter_turn` property in `myrobot.urdf.xacro`), not exactly π/2. When a cylinder lies exactly flat, Gazebo handles its ground contact poorly and the wheels slip.

## Roadmap

- Handling failed deliveries (report back, or retry later)
- Simulated battery with automatic docking and charging
- Unit tests, launch tests and CI

## License

Apache 2.0, see `src/LICENSE`.
