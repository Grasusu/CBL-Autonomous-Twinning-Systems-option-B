# CBL Option B Digital Twin

Simulation-only fallback repo for the 2IRR10 final demo.

Option B means there is no physical TurtleBot3. The Gazebo TurtleBot3 is treated as the physical stand-in/source of truth, and the ROS 2 digital twin nodes mirror its state, simulate plant health inspection, publish results, and log evidence.

## Entities

- Physical stand-in: TurtleBot3 Burger in Gazebo inside `my_tb3_world`.
- Digital entity: `tb3_pesticide_dt` nodes, especially `inspection_twin_node`, `/dt/digital/mission_state`, `/dt/digital/inspection_result`, and RViz markers.

## What The Demo Shows

- Gazebo TurtleBot3 autonomously navigates to predefined plant zones with Nav2.
- At each plant, the mission waits to simulate hyperspectral plant stress/disease inspection.
- The digital twin publishes `OK` or `TREATMENT_NEEDED`.
- Treatment-needed plants include `recommendation: APPLY_PESTICIDE`.
- Gazebo `/scan` is converted into `/dt/physical/environment_state` and mirrored inside `/dt/digital/mission_state`.
- The digital side can publish `/dt/digital/control` to inject a camera health state change.
- `/dt/physical/inspection_log` publishes per-plant evidence and the mission summary.

## Rubric Evidence

### 1. Bidirectional Pub/Sub

Physical stand-in to digital side:

```text
/dt/physical/mission_state       -> inspection_twin_node subscribes
/dt/physical/inspection_request   -> inspection_twin_node subscribes
/dt/physical/environment_state    -> inspection_twin_node subscribes
```

Digital side to physical stand-in:

```text
/dt/digital/mission_state         -> plant_nav2_mission_node subscribes
/dt/digital/inspection_result     -> plant_nav2_mission_node subscribes
/dt/digital/control               -> inspection_twin_node subscribes for digital fault injection
```

Evidence commands:

```bash
ros2 topic list | grep /dt
ros2 topic echo /dt/physical/mission_state
ros2 topic echo /dt/digital/mission_state
ros2 topic echo /dt/digital/inspection_result
```

### 2. State Synchronization

The digital entity mirrors mission mode, active zone, camera health, latest inspection result, safety state, and environment state.

Inject a digital-side camera fault:

```bash
ros2 topic pub --once /dt/digital/control std_msgs/msg/String \
  "{data: '{\"camera_health\":\"degraded\"}'}"
```

Then watch it appear in the digital state and later inspection logs:

```bash
ros2 topic echo /dt/digital/mission_state
ros2 topic echo /dt/physical/inspection_log
```

Reset it:

```bash
ros2 topic pub --once /dt/digital/control std_msgs/msg/String \
  "{data: '{\"camera_health\":\"healthy\"}'}"
```

### 3. Environmental Interaction

The Gazebo stand-in publishes `/scan`; `option_b_environment_node` converts this into `/dt/physical/environment_state`; `inspection_twin_node` mirrors that into `/dt/digital/mission_state`.

Evidence commands:

```bash
ros2 topic echo /dt/physical/environment_state
ros2 topic echo /dt/digital/mission_state
```

When the robot approaches a wall or obstacle, `min_front_m` changes and `front_obstacle` becomes true when the front distance is below the configured threshold. Nav2 uses the same simulated LIDAR/costmap to plan around obstacles.

## Repo Contents

- `tb3_pesticide_dt`: mission nodes, digital twin nodes, configs, maps, launch files, and runbook.
- `my_tb3_world`: Gazebo arena world used by the simulation.

## Build In Docker / ROS 2 Workspace

Clone this repo into the `src` folder of a ROS 2 workspace:

```bash
cd /ws/src
git clone <OPTION_B_REPO_URL> cbl_option_b

cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

If you are copying the folder manually, copy the whole `CBL-Option-B-Digital-Twin` folder into `/ws/src/cbl_option_b`, then run the same build commands from `/ws`.

## Run Option B Demo

Use separate terminals. Source the workspace in each one:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

### Terminal 1: Gazebo Physical Stand-In

```bash
ros2 launch tb3_pesticide_dt pesticide_world.launch.py gui:=true
```

Use `gui:=false` if Gazebo GUI is too heavy and you only need headless simulation.

### Terminal 2: Nav2

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=/ws/src/cbl_option_b/tb3_pesticide_dt/maps/map.yaml
```

In RViz, set the initial pose at the Gazebo start location if AMCL is not aligned.

### Terminal 3: Mission + Digital Twin

```bash
ros2 launch tb3_pesticide_dt pesticide_nav2_dt.launch.py \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_plant_zones.yaml \
  use_sim_time:=true
```

### Terminal 4: Evidence Log

```bash
ros2 topic echo /dt/physical/inspection_log
```

You should see `INSPECTION_LOG` messages for each plant and a final `MISSION_SUMMARY`.

### Terminal 5: Environment Evidence

```bash
ros2 topic echo /dt/physical/environment_state
```

### Terminal 6: Digital State Evidence

```bash
ros2 topic echo /dt/digital/mission_state
```

## Notes

- Keep this repo separate from the main Option A repo.
- Do not clone both this repo and the original `tb3_pesticide_dt` package into the same workspace `src` unless you remove/rename one of them, because they contain the same ROS package names.
- The full package runbook is inside `tb3_pesticide_dt/README.md`.
