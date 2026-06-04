# TurtleBot3 Plant Health Digital Twin

ROS 2 Jazzy proof of concept for the 2IRR10 final digital twin demo.

The robot autonomously visits predefined plant zones in the arena, waits to simulate an inspection, asks a digital twin node for a simulated hyperspectral plant stress/disease result, logs each plant as `OK` or `TREATMENT_NEEDED`, and recommends pesticide application when treatment is needed. It then returns to the calibrated home/start location.

The final tested demo uses Nav2 for the full route:

```text
plant_nav2_mission_node -> NavigateToPose goals -> Nav2 -> /cmd_vel -> Gazebo TurtleBot3
inspection_twin_node     -> simulated hyperspectral camera results
arena_map_node           -> optional RViz/digital arena markers
```

For the final physical robot demo, Gazebo also runs as the live visual digital twin:

```text
real TurtleBot3 /odom or /amcl_pose -> gazebo_pose_mirror_node -> Gazebo burger model pose
```

## What This Demonstrates

- Autonomous navigation through 8 plant inspection zones.
- A digital twin entity synchronized with the physical/simulation side.
- Bidirectional pub/sub between physical and digital sides.
- Simulated hyperspectral plant stress/disease classification.
- Inspection logs with `OK` and `TREATMENT_NEEDED` results.
- A final autonomous return to a calibrated `plant_home` waypoint.
- Required live Gazebo pose mirroring for the physical robot final demo.
- Reuse of the course scanner/safety-node idea through the included `twin_safety_node` and hybrid fallback code.

## Rubric Mapping

1. **Bidirectional pub/sub**
   - Physical to digital: `/dt/physical/mission_state`, `/dt/physical/inspection_request`
   - Digital to physical: `/dt/digital/mission_state`, `/dt/digital/inspection_result`

2. **State synchronization**
   - Mission mode, active plant zone, digital camera health, inspection result, and final status are mirrored through `/dt/physical/mission_state` and `/dt/digital/mission_state`.
   - For the physical robot demo, `gazebo_pose_mirror_node` can mirror the real robot pose into Gazebo so the digital robot follows the real one visually.

3. **Environmental interaction**
   - The robot moves through the arena, reaches plant zones, waits at each plant, receives a plant health classification, and returns home.

4. **Evidence/logging**
   - `/dt/physical/inspection_log` publishes per-plant logs and the final mission summary.

## Package Contents

- `plant_nav2_mission_node`: final full-Nav2 mission controller.
- `inspection_twin_node`: digital twin that simulates the hyperspectral camera.
- `gazebo_pose_mirror_node`: copies real robot pose into the Gazebo model for live digital-twin visualization.
- `arena_map_node`: publishes optional digital markers for the arena.
- `twin_safety_node`: scanner-based safety bridge from the earlier mini-project pattern.
- `plant_mission_node`: hybrid/fallback mission controller.
- `config/nav2_plant_zones.yaml`: final full-Nav2 plant route and calibrated `plant_home`.
- `config/real_robot_lab.yaml`: physical robot mission config with longer timeouts, manual RViz initial pose, and captured return-home pose.
- `maps/map.yaml`: university-provided map from `mapFiles.zip`; try this first on the real robot.
- `maps/gazebo_slam_map.yaml`: backup Gazebo SLAM map; try this second if the university map does not align.
- `launch/pesticide_world_visual_twin.launch.py`: Gazebo arena plus visual robot only, without fake ROS `/scan` or `/odom` bridge.

## Install / Build

Inside the course Docker workspace:

```bash
cd /ws/src
git clone https://github.com/LarsieVL/S.M.A.R.T.L.E..git tb3_pesticide_dt

cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install
source install/setup.bash
```

If the package is already copied into `/ws/src/tb3_pesticide_dt`, only run the build commands.

On a lab laptop without Docker, put the project inside the `src` folder of a ROS 2 workspace:

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/LarsieVL/S.M.A.R.T.L.E..git tb3_pesticide_dt

cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tb3_pesticide_dt --symlink-install
source install/setup.bash
```

So yes: on the lab laptop, clone/copy this repository into `~/turtlebot3_ws/src/tb3_pesticide_dt` unless your course uses a different workspace name.

For Gazebo visualization, this project also expects the course `my_tb3_world` package in the same workspace, because that package contains `new_world.world`. The structure should be:

```text
~/turtlebot3_ws/src/
  my_tb3_world/
  tb3_pesticide_dt/
```

## Final Tested Gazebo Demo

Important: run each launch only once. If the robot behaves strangely, restart the container first:

```bash
docker restart turtlebot3_container
```

Open 4 Docker terminals:

```bash
docker exec -it turtlebot3_container bash
```

### Terminal 1: Gazebo

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch tb3_pesticide_dt pesticide_world.launch.py gui:=true
```

### Terminal 2: Nav2

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=/ws/src/tb3_pesticide_dt/maps/map.yaml \
  rviz:=false
```

Wait 10 to 15 seconds before starting the mission.

### Terminal 3: Evidence Logs

Start this before the mission:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 topic echo /dt/physical/inspection_log std_msgs/msg/String --full-length
```

### Terminal 4: Mission

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 launch tb3_pesticide_dt pesticide_nav2_dt.launch.py \
  params_file:=/ws/src/tb3_pesticide_dt/config/nav2_plant_zones.yaml \
  use_sim_time:=true
```

Expected final lines:

```text
Route waypoint 9 is plant_home
Sent Nav2 return goal plant_home: Home / Start at {'x': -0.8, 'y': -0.07, 'yaw': 0.0}
Plant inspection route complete: RETURNED_HOME
```

Check the final Gazebo pose:

```bash
gz model -m burger -p
```

It should be close to Gazebo/world `x=0`, `y=0`. The Nav2 `plant_home` is `x=-0.80`, `y=-0.07` because the Nav2 map frame and Gazebo world frame have a small offset.

## Useful Evidence Commands

```bash
ros2 topic list | grep /dt
ros2 topic echo /dt/physical/mission_state std_msgs/msg/String --full-length
ros2 topic echo /dt/digital/mission_state std_msgs/msg/String --full-length
ros2 topic echo /dt/physical/inspection_request std_msgs/msg/String --full-length
ros2 topic echo /dt/digital/inspection_result std_msgs/msg/String --full-length
ros2 topic echo /dt/physical/inspection_log std_msgs/msg/String --full-length
```

Optional camera-health state sync demo:

```bash
ros2 param set /inspection_twin_node camera_health degraded
ros2 topic echo /dt/digital/mission_state std_msgs/msg/String --full-length
ros2 param set /inspection_twin_node camera_health healthy
```

## Changing Plant Zones

Edit:

```text
config/nav2_plant_zones.yaml
```

The first 8 entries are inspected plants. The 9th entry, `plant_home`, is the final return waypoint and is not inspected.

Keep all arrays the same length:

```yaml
zone_ids
zone_names
zone_x
zone_y
zone_yaw
zone_plant_stress_indices
zone_expected_statuses
```

## Lab Day Step-By-Step Runbook

Use this on the lab laptop for the physical robot test. The only command that runs on the robot itself is the TurtleBot3 bringup over SSH. Everything else runs in normal terminals on the lab laptop.

### 0. Prepare The Two ROS Packages

The lab workspace needs two package folders:

```text
tb3_pesticide_dt/
my_tb3_world/
```

`tb3_pesticide_dt` contains the mission nodes, Nav2 route, logs, Gazebo pose mirror, and maps:

```text
tb3_pesticide_dt/maps/map.yaml
tb3_pesticide_dt/maps/map.pgm
tb3_pesticide_dt/maps/gazebo_slam_map.yaml
tb3_pesticide_dt/maps/gazebo_slam_map.pgm
```

`my_tb3_world` contains the Gazebo arena:

```text
my_tb3_world/worlds/new_world.world
```

Copy both package folders from the development machine, USB drive, or shared storage into the lab laptop's ROS workspace `src` folder.

### 1. Normal Lab Laptop Terminal: Copy Packages Into The Workspace

Target structure:

```text
~/turtlebot3_ws/src/
  tb3_pesticide_dt/
  my_tb3_world/
```

If old folders with the same names already exist, rename them first:

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
mv tb3_pesticide_dt tb3_pesticide_dt_old 2>/dev/null || true
mv my_tb3_world my_tb3_world_old 2>/dev/null || true
```

Then copy `tb3_pesticide_dt` and `my_tb3_world` into:

```text
~/turtlebot3_ws/src/
```

If you use Git for `tb3_pesticide_dt`, run:

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/LarsieVL/S.M.A.R.T.L.E..git tb3_pesticide_dt
```

You still need to copy or install `my_tb3_world` separately for Gazebo.

### 2. Normal Lab Laptop Terminal: Build The Packages

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash

colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install
source install/setup.bash
```

Verify both packages exist:

```bash
ros2 pkg list | grep tb3_pesticide_dt
ros2 pkg list | grep my_tb3_world
```

### 3. SSH Terminal On The Robot: Start TurtleBot3 Bringup

Open an SSH terminal from the lab laptop to the robot:

```bash
ssh ubuntu@ROBOT_IP
```

Replace `ROBOT_IP` with the robot IP used in the lab. Then run this inside the SSH terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

ros2 launch turtlebot3_bringup robot.launch.py
```

Leave this SSH terminal running. If the course uses a different `ROS_DOMAIN_ID`, use that same ID on both the robot and the lab laptop.

### 4. Normal Lab Laptop Terminal: Verify Robot Topics

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=burger

ros2 topic list
ros2 topic echo /scan
```

If `/scan` publishes data, stop it with `Ctrl+C`. Also check odometry:

```bash
ros2 topic echo /odom
```

Stop it with `Ctrl+C`. Do not continue until `/scan` and `/odom` work.

### 5. Normal Lab Laptop Terminal: Start Nav2 With The University Map

Use the university map first:

```text
~/turtlebot3_ws/src/tb3_pesticide_dt/maps/map.yaml
```

Start Nav2:

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  map:=$HOME/turtlebot3_ws/src/tb3_pesticide_dt/maps/map.yaml
```

In RViz:

1. Use `2D Pose Estimate` to place the robot where it really is.
2. Check that `/scan` lines up with the arena walls.
3. Send one short `2D Nav Goal`.
4. Continue only if the robot moves sensibly and stops at the goal.

If the university map does not align, stop Nav2 with `Ctrl+C` and try the backup Gazebo SLAM map:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  map:=$HOME/turtlebot3_ws/src/tb3_pesticide_dt/maps/gazebo_slam_map.yaml
```

If neither map aligns, make a new map with the physical robot before running the mission.

The current `config/real_robot_lab.yaml` route uses the same plant coordinates as the university/Gazebo map. Try it first after the short RViz goal works. If the robot drives to the wrong plant positions in the real arena, recalibrate the waypoint values in that YAML file.

### 6. Normal Lab Laptop Terminal: Make A New Map Only If Needed

Run Cartographer:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false
```

In another normal lab laptop terminal, teleop slowly around the arena:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger

ros2 run turtlebot3_teleop teleop_keyboard
```

Save the physical map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/arena_map
```

Then start Nav2 with:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  map:=$HOME/arena_map.yaml
```

If you make a new physical map, recalibrate the plant waypoint coordinates before running the full mission.

To read the robot pose at a plant location:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Edit the route:

```bash
nano ~/turtlebot3_ws/src/tb3_pesticide_dt/config/real_robot_lab.yaml
```

Update `zone_x`, `zone_y`, `zone_yaw`, `home_x`, `home_y`, and `home_yaw`. The ninth waypoint, `plant_home`, is the final return position.

### 7. Normal Lab Laptop Terminal: Start The Required Gazebo Visual Twin

Do not use `pesticide_world.launch.py` with the real robot. That launch bridges simulated `/scan`, `/odom`, and `/tf`, which can conflict with the real robot. For the physical demo, use the visual-only launch:

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch tb3_pesticide_dt pesticide_world_visual_twin.launch.py gui:=true
```

### 8. Normal Lab Laptop Terminal: Start Real Robot To Gazebo Sync

Start with `/odom`:

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

ros2 launch tb3_pesticide_dt gazebo_pose_mirror.launch.py \
  source_topic:=/odom \
  source_type:=odom \
  model_name:=burger \
  world_name:=default
```

If the Gazebo robot follows poorly and `/amcl_pose` is available, stop the mirror and use AMCL pose instead:

```bash
ros2 launch tb3_pesticide_dt gazebo_pose_mirror.launch.py \
  source_topic:=/amcl_pose \
  source_type:=amcl_pose \
  model_name:=burger \
  world_name:=default
```

### 9. Normal Lab Laptop Terminal: Start Evidence Logs

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

ros2 topic echo /dt/physical/inspection_log std_msgs/msg/String --full-length
```

### 10. Normal Lab Laptop Terminal: Start The Mission

Run this only after:

1. `/scan` and `/odom` work.
2. Nav2 localizes on the map.
3. One short RViz `2D Nav Goal` works.
4. Gazebo visual twin is open.
5. `gazebo_pose_mirror_node` is running.

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=burger

ros2 launch tb3_pesticide_dt pesticide_nav2_dt.launch.py \
  params_file:=~/turtlebot3_ws/src/tb3_pesticide_dt/config/real_robot_lab.yaml \
  use_sim_time:=false
```

Expected finish:

```text
Route waypoint 9 is plant_home
Sent Nav2 return goal plant_home
Plant inspection route complete: RETURNED_HOME
```

### 11. Lab Troubleshooting Notes

If the first plant is skipped instantly, do not launch the mission yet. Set the initial pose in RViz, wait until the scan lines up with the map, and send one short RViz `2D Nav Goal` first. The physical config uses `publish_initial_pose: false` so the mission does not overwrite your RViz pose.

If the robot drives slowly, keep the longer physical timeout. `config/real_robot_lab.yaml` uses:

```yaml
goal_timeout_s: 300.0
```

If the Gazebo robot is synchronized but starts outside the arena, stop the pose mirror, place the real robot at the physical start, restart the robot bringup so `/odom` starts from the start pose, then start `pesticide_world_visual_twin.launch.py` and `gazebo_pose_mirror.launch.py` again.

If final return goes to the wrong place, use `config/real_robot_lab.yaml`. It captures the real start pose after RViz localization settles:

```yaml
use_captured_home_pose: true
```

If the robot hits a wall or approaches walls too closely, recalibrate the plant waypoint coordinates farther from the walls in `config/real_robot_lab.yaml`.

### 12. What To Say In The Demo

- The real TurtleBot3 navigates autonomously to predefined plant zones using Nav2.
- At each plant, it waits to simulate hyperspectral plant stress/disease inspection.
- The digital twin node simulates the hyperspectral camera and returns `OK` or `TREATMENT_NEEDED`; treatment-needed plants include `recommendation: APPLY_PESTICIDE`.
- `/dt/physical/inspection_log` is the evidence topic for inspection results and mission summary.
- Gazebo is the live visual digital twin because it mirrors the real robot pose from `/odom` or `/amcl_pose`.

## Real Robot Lab Setup

Use the `Lab Day Step-By-Step Runbook` above for the physical robot. The lab laptop commands intentionally do not source `/opt/turtlebot3_ws/install/setup.bash`, because that workspace may not exist on the lab machine. Source only:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

after building from `~/turtlebot3_ws`.

### Real Robot Safety Notes

- Test with only 1 or 2 waypoints first before running the whole route.
- Keep a hand near the robot or be ready to stop the launch with `Ctrl+C`.
- Run in a clear arena and keep people out of the robot path.
- Recheck localization in RViz if the robot starts navigating to the wrong place.
- The simulated hyperspectral camera result is still generated by `inspection_twin_node`; no real camera hardware is required for this proof of concept.

## GitHub Setup

From this package folder:

```bash
cd <workspace>/src/tb3_pesticide_dt
git status
git add .
git commit -m "Add TurtleBot3 pesticide inspection digital twin demo"
git branch -M main
git remote add origin <YOUR_GITHUB_REPO_URL>
git push -u origin main
```

If `origin` already exists:

```bash
git remote set-url origin <YOUR_GITHUB_REPO_URL>
git push -u origin main
```
