# CBL Option B Digital Twin

Final simulation-only TurtleBot3 digital twin demo for the 2IRR10 course.

This is the official Option B implementation: there is no physical TurtleBot3. A Gazebo TurtleBot3 acts as the physical stand-in/source of truth, while a ROS 2 digital control panel plus digital twin nodes mirror state, inspect plant zones, publish plant-health results, and record rubric evidence.

## What The Demo Shows

- The Gazebo TurtleBot3 autonomously navigates through 6 plant inspection zones with Nav2.
- A digital control panel publishes `/dt/digital/dashboard`, sends control commands on `/dt/digital/control`, and is viewed through a browser dashboard.
- The robot stops beside each plant, faces it, and the on-board (simulated) camera captures a hyperspectral plant stress/disease reading that the twin analyses.
- The digital twin returns `OK` or `TREATMENT_NEEDED`; treatment-needed plants recommend `APPLY_PESTICIDE`.
- Gazebo `/scan` is converted into `/dt/physical/environment_state` and mirrored into `/dt/digital/mission_state`.

## Rubric Mapping

### 1. Bidirectional Pub/Sub

Physical stand-in to digital side:

```text
/dt/physical/mission_state
/dt/physical/inspection_request
/dt/physical/environment_state
```

Digital side to physical stand-in:

```text
/dt/digital/mission_state
/dt/digital/inspection_result
/dt/digital/control
/dt/digital/dashboard
/dt/digital/dashboard_summary
```

The mission node consumes digital inspection results before moving to the next plant, so the route is not just a one-way animation. The dashboard also publishes an initial camera-health command on `/dt/digital/control`, proving that the digital entity can affect the twin state.

### 2. State Synchronization

Mirrored state includes:

```text
mission mode
active plant zone
AMCL/map pose
camera health
latest inspection result
environment state
```

Optional fault injection:

```bash
ros2 topic pub --once /dt/digital/control std_msgs/msg/String \
  "{data: '{\"camera_health\":\"degraded\"}'}"
```

Reset:

```bash
ros2 topic pub --once /dt/digital/control std_msgs/msg/String \
  "{data: '{\"camera_health\":\"healthy\"}'}"
```

### 3. Environmental Interaction

The stand-in robot uses Gazebo LIDAR for Nav2 planning. The same `/scan` stream is converted into `/dt/physical/environment_state`, then mirrored by the digital twin. During the route, `min_front_m`, `front_obstacle`, and `environment_mode` change as the robot approaches walls and arena obstacles. The plant models are visual inspection targets, so they make the concept clear without trapping Nav2 in decorative collision geometry.

## Repository Layout

```text
CBL-Option-B-Digital-Twin/
  my_tb3_world/          Gazebo world with the arena and visual plant markers
  tb3_pesticide_dt/      ROS 2 package with mission, twin, dashboard, maps, launch files
```

Important files:

```text
tb3_pesticide_dt/config/nav2_plant_zones.yaml       plant route and inspection settings
tb3_pesticide_dt/config/nav2_burger_option_b.yaml   Nav2 params tuned for Gazebo Option B
tb3_pesticide_dt/tb3_pesticide_dt/option_b_dashboard_node.py
tb3_pesticide_dt/tb3_pesticide_dt/option_b_dashboard_web.py
tb3_pesticide_dt/launch/pesticide_world.launch.py
tb3_pesticide_dt/launch/option_b_navigation2.launch.py
tb3_pesticide_dt/launch/pesticide_nav2_dt.launch.py
```

## Setup In Docker

From a Mac terminal:

```bash
colima start --cpu 4 --memory 8 --runtime docker
docker context use colima

mkdir -p "$HOME/option_b_ws/src"

rsync -a --delete \
  "$HOME/CBL-Option-B-Digital-Twin/" \
  "$HOME/option_b_ws/src/cbl_option_b/"

docker rm -f turtlebot3_container 2>/dev/null || true

docker run --rm -it \
  -p 5901:5901 \
  -p 8080:8080 \
  -v "$HOME/option_b_ws:/ws" \
  --name turtlebot3_container \
  turtlebot3_ws_vnc
```

Leave that terminal open.

In a second Mac terminal, start VNC:

```bash
docker exec turtlebot3_container bash -lc \
'rm -f /tmp/.X1-lock /tmp/.X11-unix/X1; vncserver -kill :1 2>/dev/null || true; vncserver :1 -geometry 1280x800 -depth 24 -localhost no -rfbport 5901'
```

Open VNC at `127.0.0.1:5901`, password `ros`.

## Build

Open a Docker terminal:

```bash
docker exec -it turtlebot3_container bash
```

Inside Docker:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:1
```

## Final Demo Commands

Recommended demo flow: one launch terminal for the full robot mission and one dashboard terminal. In every Docker terminal, start with:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:1
```

### Terminal 1: Full Robot Mission

```bash
ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=true
```

This starts Gazebo, Nav2, the AMCL initial pose publisher, the plant mission, and the digital dashboard/control node. The launch intentionally waits about one minute before the robot moves so Nav2 and localization are stable before the first goal.

Expected route:

```text
plant_a -> plant_b -> plant_c -> plant_d -> plant_e -> plant_f -> plant_home
```

The plants are small visual markers. The robot stops at scan poses about 0.55 m from each marker, waits for inspection, logs the plant health result, then continues.

### Terminal 2: Browser Dashboard

```bash
ros2 run tb3_pesticide_dt option_b_dashboard_web
```

Then open `http://127.0.0.1:8080` in a browser on the host. It subscribes to `/dt/digital/dashboard` and shows the important demo information (current mode, active plant, the robot's camera health, latest plant-health result, inspection history, environment state) and can drive the demo: inject a camera fault, drop/remove an obstacle, echo topics, and show node wiring.

### Optional Terminal 3: Topic Echoes

The browser dashboard already exposes these live, but you can also echo them directly:

```bash
ros2 topic echo /dt/digital/dashboard_summary std_msgs/msg/String --full-length
ros2 topic echo /dt/physical/inspection_log std_msgs/msg/String --full-length
ros2 topic echo /dt/physical/environment_state std_msgs/msg/String --full-length
ros2 topic echo /dt/digital/mission_state std_msgs/msg/String --full-length
ros2 topic echo /dt/digital/control std_msgs/msg/String --full-length
```

### Manual Fallback

Use this only if you want to debug each subsystem separately.

```bash
ros2 launch tb3_pesticide_dt pesticide_world.launch.py gui:=true
```

```bash
ros2 launch tb3_pesticide_dt option_b_navigation2.launch.py \
  use_sim_time:=true \
  map:=/ws/src/cbl_option_b/tb3_pesticide_dt/maps/map.yaml \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_burger_option_b.yaml
```

```bash
ros2 run tb3_pesticide_dt nav2_initial_pose_node --ros-args \
  -p use_sim_time:=true \
  -p x:=-0.80 \
  -p y:=-0.07 \
  -p yaw:=0.0 \
  -p duration_s:=14.0
```

```bash
ros2 launch tb3_pesticide_dt pesticide_nav2_dt.launch.py \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_plant_zones.yaml \
  use_sim_time:=true
```

## What To Point Out In The Presentation

Open the browser dashboard (`http://127.0.0.1:8080`) and walk through the three rubric items:

- Bidirectional pub/sub: the robot's camera sends its reading (`/dt/physical/inspection_request`) and the twin returns its analysis (`/dt/digital/inspection_result`). Use *Show node wiring* and *Echo request* / *Echo result*.
- State synchronization: the physical and digital cards mirror mission mode and the robot's camera health. *Inject fault: degraded* flips the camera state on both sides and lowers the next plant's confidence.
- Environmental interaction: *Drop obstacle on route* spawns a box that is not in the map; the robot's LIDAR detects it, the environment card flips to `OBSTACLE_AHEAD`, and Nav2 reroutes around it.

Expected mission behavior:

```text
plant_a -> OK
plant_b -> TREATMENT_NEEDED
plant_c -> OK
plant_d -> TREATMENT_NEEDED
plant_e -> TREATMENT_NEEDED
plant_f -> OK
plant_home -> RETURNED_HOME
```

## Optional One-Terminal Demo

This is convenient for a quick screen recording, but the multi-terminal flow above is safer for debugging.

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:1

ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=true
```

This starts only the main Gazebo robot. The digital entity is the ROS dashboard/control panel, not a second visual robot.

## Troubleshooting

If `Package tb3_pesticide_dt not found`, rebuild and source:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install
source install/setup.bash
```

If Nav2 prints collision monitor timestamp warnings, make sure Terminal 3 uses:

```bash
ros2 launch tb3_pesticide_dt option_b_navigation2.launch.py ...
```

Do not use the default `turtlebot3_navigation2 navigation2.launch.py` for the final Option B demo.

If Docker says the container does not exist, recreate it with the setup command above. `docker restart turtlebot3_container` only works after the container has already been created.

If Gazebo GUI fails, restart VNC and open `127.0.0.1:5901` again.

## Verification Already Run

The repo was checked with:

```bash
python3 -m compileall
colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install
colcon test --packages-select tb3_pesticide_dt
ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=false
```

At the time of this update, Docker build passed, package tests passed, and a full headless end-to-end run completed all 6 plant inspections and returned to `plant_home` with `RETURNED_HOME`.
