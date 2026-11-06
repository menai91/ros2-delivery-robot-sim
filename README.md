# ROS 2 Autonomous Mobile Robot (Gazebo Simulation)

A simulated differential-drive robot that maps an environment with SLAM, navigates it with Nav2, and runs missions through a C++ behavior-tree layer built on BehaviorTree.CPP. Built with ROS 2 Jazzy and Gazebo Harmonic. Simulation only.

https://github.com/user-attachments/assets/6ad3dcd4-9123-432f-8862-518ca15df85e

## What it does

The robot runs in one of three modes:

1. **Mapping:** drive the robot around an unknown room with teleop while `slam_toolbox` builds a 2D map from lidar, then save the map to a file.
2. **Navigation:** load the saved map, click a goal in RViz, and Nav2 plans a path and drives the robot there.
3. **Missions:** `bt_runner` loads a behavior tree from an XML file and drives the robot through it. Two trees are included:
   - `single_delivery.xml` runs one delivery: drive to a pickup station, load, drive to a dropoff station, unload. Then it exits.
   - `delivery_queue.xml` keeps running and works through delivery requests sent to the `/delivery_request` service, one after another. When there is nothing to do, it waits and checks again every second. If a delivery fails, it retries, takes the item back to the pickup station if it is holding one, and then either moves the task to the back of the queue or discards it.

   Loading and unloading are stubs for now: the robot waits 5 s at each station.

## Architecture

| Package | Folder | Contents |
| --- | --- | --- |
| `ros2_project` | `src/robot_description/` | Robot model (URDF, written as xacro), Gazebo world, launch file, saved map, Nav2 and SLAM configuration |
| `amr_navigation` | `src/amr_navigation/` | `Navigator`: a non-blocking C++ wrapper around Nav2's `navigate_to_pose` action. A YAML loader for named station poses. No behavior-tree dependency. |
| `amr_interfaces` | `src/amr_interfaces/` | The `DeliveryRequest` service and the `Location` message it uses |
| `amr_autonomy` | `src/amr_autonomy/` | Behavior-tree nodes (`MoveToPosition`, `GetNextTask`, `SetTaskOutcome`, `IsItemOnBoard`, `ClearPayload`, plus `PickupItem` and `DropoffItem` stubs), node registration, mission trees (`trees/`), the task queue, the `Payload` record of what the robot carries, the `/delivery_request` server, station poses (`config/stations.yaml`), a launch file, and the `bt_runner` executable |

Design notes:

- **Non-blocking navigation.** A behavior tree must return from every tick quickly, so `Navigator` exposes `start` / `get_status` / `cancel` instead of a blocking "drive to pose" call. The `MoveToPosition` tree node is a thin adapter that polls it each tick.
- **Stale-goal guard.** Cancelling an action is asynchronous, so callbacks for an old goal can still arrive, including a goal that Nav2 accepts after it was cancelled. Every callback carries a goal ID and ignores itself if a newer goal has started; a late accept for a cancelled goal is cancelled on arrival.
- **Goal-response timeout.** If Nav2 never answers a goal, `Navigator` fails it after a timeout instead of waiting forever. Limits on total drive time belong to the tree (`Timeout` decorator), where they are visible and set per task.
- **Failure policy as tree structure.** Retries, timeouts and when to give up are written in the XML, not in C++, so they are visible and can be set per task. Each move gets up to 2 attempts of 60 s each (3 in `single_delivery.xml`). `MoveToPosition` writes Nav2's error code to the blackboard. Errors that retrying the same goal right away cannot fix (204, goal outside the map; 208, no valid path) end the move's retries at once. If a move still fails, the delivery attempt fails instead of continuing to the dropoff without the item. Nav2 still runs its own recovery behaviors inside each attempt; the tree only makes mission-level choices.
- **Failed deliveries are retried, then re-queued.** `delivery_queue.xml` handles failure at three levels, each on its own time scale: a move is retried, a load or unload step is retried once, and the whole delivery is tried up to 3 times. If the robot is holding the item when an attempt fails, it first takes the item back to the pickup station, so every attempt, and every task that comes back later, starts at pickup with an empty robot. After 3 failed attempts the task goes to the back of the queue, so other tasks run first and a temporary problem (such as a blocked doorway) has time to clear. A goal outside the map (error 204) can never succeed, so that task is discarded instead. If the item can't be returned either, the robot pauses, assumes a person removed it, and discards the task (see Known limitations).
- **Task state and robot state are kept apart.** The queue keeps the task it handed out until the tree records how it ended (completed, discarded or re-queued with the same ID), so a task can't be lost between being taken and finishing. Whether the robot is carrying an item is robot state, held in a separate `Payload` object that only the load and unload nodes change. Like `Navigator`, both are plain C++ objects owned by `main` and passed to the tree nodes that need them.
- **The tree's vocabulary is typed.** Values the XML scripts compare, such as how a task's attempts ended, are C++ enums registered with BT.CPP's scripting language. A misspelled name stops `bt_runner` with an error instead of silently comparing false.
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
  "{pickup: {x: 4.0, y: -3.35, yaw: 1.57}, dropoff: {station: room2_station}}"
```

The reply says whether the request was accepted, and why not if it was rejected. Station poses are read from the installed `config/stations.yaml` when `bt_runner` starts, so rebuild and restart it after editing the file.

### Testing failure handling

The load and unload steps can be forced to fail with BT.CPP's built-in `_failureIf` attribute, without changing any C++. Add it to a node in `src/amr_autonomy/trees/delivery_queue.xml`, for example the unload at the dropoff station:

```xml
<DropoffItem name="UnloadAtDropoff" _failureIf="true"/>
```

Then run that file directly, so the edit is used without rebuilding:

```bash
ros2 run amr_autonomy bt_runner src/amr_autonomy/trees/delivery_queue.xml
```

A forced node fails without running, so it leaves no log line of its own; it shows in what the robot does next. A pose outside the map, such as `dropoff: {x: 20.0, y: 0.0, yaw: 0.0}`, tests the error 204 path.

| Scenario | Outcome |
| --- | --- |
| Pickup pose outside the map | Task discarded at once, no retries |
| Dropoff pose outside the map | Item taken back to the pickup station, task discarded |
| `UnloadAtDropoff` forced to fail | Item taken back to pickup on each of 3 attempts, then the task is re-queued at the back. If it is the only task, it starts again straight away |
| `UnloadAtDropoff` and `UnloadOnReturn` (the unload in the return) forced to fail | Item can't be returned: 5 s pause, payload cleared, task discarded |

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

Robot log. The second request and the rejected one arrive while goal #1 is still driving, and the robot starts the second delivery as soon as the first one finishes:

```text
[bt_runner]: Connected to 'navigate_to_pose' action server
[bt_runner]: Accepting delivery requests on '/delivery_request' (2 stations loaded)
[bt_runner]: Loaded behavior tree from 'src/amr_autonomy/trees/delivery_queue.xml'
[bt_runner]: Queued delivery: pickup (5.35, -3.35, 3.14) -> dropoff (-1.00, -3.35, 1.57), 1 task(s) waiting
[GetNextTask]: [GetNextTask] Starting task #1: pickup (5.35, -3.35, 3.14) -> dropoff (-1.00, -3.35, 1.57)
[bt_runner]: Sending goal #1: x=5.35, y=-3.35, yaw=3.14
[bt_runner]: Goal #1 accepted, navigating
[bt_runner]: Queued delivery: pickup (5.35, -3.35, 3.14) -> dropoff (-1.00, -3.35, 1.57), 1 task(s) waiting
[bt_runner]: Rejected delivery request: unknown dropoff station 'room22_station'
[bt_runner]: Goal #1 reached
[PickupItem]: [LoadAtPickup] Started
[PickupItem]: [LoadAtPickup] Finished, item loaded
[bt_runner]: Sending goal #2: x=-1.00, y=-3.35, yaw=1.57
[bt_runner]: Goal #2 accepted, navigating
[bt_runner]: Goal #2 reached
[DropoffItem]: [UnloadAtDropoff] Started
[DropoffItem]: [UnloadAtDropoff] Finished, item unloaded
[TaskQueue]: Task #1 completed
[GetNextTask]: [GetNextTask] Starting task #2: pickup (5.35, -3.35, 3.14) -> dropoff (-1.00, -3.35, 1.57)
[bt_runner]: Sending goal #3: x=5.35, y=-3.35, yaw=3.14
[bt_runner]: Goal #3 accepted, navigating
[bt_runner]: Goal #3 reached
[PickupItem]: [LoadAtPickup] Started
[PickupItem]: [LoadAtPickup] Finished, item loaded
[bt_runner]: Sending goal #4: x=-1.00, y=-3.35, yaw=1.57
[bt_runner]: Goal #4 accepted, navigating
[bt_runner]: Goal #4 reached
[DropoffItem]: [UnloadAtDropoff] Started
[DropoffItem]: [UnloadAtDropoff] Finished, item unloaded
[TaskQueue]: Task #2 completed
```

</details>

<details>
<summary>A dropoff outside the map: the item is returned and the task discarded</summary>

```text
$ ros2 service call /delivery_request amr_interfaces/srv/DeliveryRequest \
    "{pickup: {station: room1_station}, dropoff: {x: 20.0, y: 0.0, yaw: 0.0}}"
```

Robot log. Nav2 rejects the dropoff goal at once with error 204, so the move is not retried. The robot takes the item back to the pickup station, and the task is discarded instead of being tried again:

```text
[bt_runner]: Queued delivery: pickup (5.35, -3.35, 3.14) -> dropoff (20.00, 0.00, 0.00), 1 task(s) waiting
[GetNextTask]: [GetNextTask] Starting task #1: pickup (5.35, -3.35, 3.14) -> dropoff (20.00, 0.00, 0.00)
[bt_runner]: Sending goal #1: x=5.35, y=-3.35, yaw=3.14
[bt_runner]: Goal #1 accepted, navigating
[bt_runner]: Goal #1 reached
[PickupItem]: [LoadAtPickup] Started
[PickupItem]: [LoadAtPickup] Finished, item loaded
[bt_runner]: Sending goal #2: x=20.00, y=0.00, yaw=0.00
[bt_runner]: Goal #2 accepted, navigating
[bt_runner]: Goal #2 aborted by Nav2, error_code=204:
[bt_runner]: Sending goal #3: x=5.35, y=-3.35, yaw=3.14
[bt_runner]: Goal #3 accepted, navigating
[bt_runner]: Goal #3 reached
[DropoffItem]: [UnloadOnReturn] Started
[DropoffItem]: [UnloadOnReturn] Finished, item unloaded
[SetTaskOutcome]: [SetTaskOutcome] Reason: goal outside the map (204)
[TaskQueue]: Task #1 failed and was discarded
```

</details>

## Known limitations

- The robot's starting pose is fixed at the map origin in `nav2_params.yaml` (`initial_pose`), which matches the spawn point in the provided world (Nav2 needs an initial pose). A different spawn point, or a map made from a different start, needs that value changed, or the pose set manually with RViz's 2D Pose Estimate.
- A task that keeps failing goes back into the queue every time. There is no overall limit, so a task that can never succeed (for example, at a broken station) is retried forever, with a trip to return the item on each attempt. Only a goal outside the map, or an item that can't be returned, ends a task as failed.
- When the item can't be returned to the pickup station, the hand-over to a person is simulated: the robot pauses for 5 s, assumes the item was removed, and discards the task. A real robot would raise an alert and wait for someone to confirm.
- The log shows goals and load and unload steps, but not the tree's own progress, such as which attempt or which phase (pickup, dropoff, return) is running.
- The queue only exists in memory. Tasks that are still waiting are lost when `bt_runner` stops.
- When a goal can't be reached, the tree does not find out right away. Nav2 can't tell a wall from an obstacle that might move, so it first spends about 17 s trying to clear the way: spinning, backing up and checking again. Only after that does it report the goal as unreachable.
- At startup, `bt_runner` waits up to 15 s for Nav2's action server to be discovered.
- The wheel collision cylinders are rotated by 1.5708 rad (the `quarter_turn` property in `myrobot.urdf.xacro`), not exactly π/2. When a cylinder lies exactly flat, Gazebo handles its ground contact poorly and the wheels slip.

## Roadmap

- An operator alert when the robot can't return an item
- Simulated battery with automatic docking and charging
- Tree-level logging and Groot2 visualisation
- Unit tests, launch tests and CI

## License

Apache 2.0, see `src/LICENSE`.
