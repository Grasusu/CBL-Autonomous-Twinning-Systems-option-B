# SMARTLE Option B Digital Twin

Simulation-only TurtleBot3 digital twin proof of concept for the 2IRR10 final demo.

This repo is the official **Option B** implementation. There is no physical TurtleBot3 in the demo. A Gazebo TurtleBot3 acts as the physical stand-in/source of truth, while ROS 2 digital twin nodes and a browser dashboard mirror state, analyze simulated plant-health readings, and expose the topic evidence for the rubric.

**Demo video:** `CBL_Group17_Demo.mp4` (included in this folder) — the 2–3 minute recording of the three required digital-twin usages.

This is a simulation-only Option B implementation, so there is no lab laptop or physical robot account: everything runs in Docker.

## What The Demo Shows

- The Gazebo TurtleBot3 autonomously navigates through 6 plant inspection zones with Nav2.
- The digital entity mirrors mission mode, active plant, pose feedback, camera health, latest inspection, and environment state.
- The robot stops beside each plant and simulates a hyperspectral plant stress/disease inspection.
- The twin returns `OK` or `TREATMENT_NEEDED`; treatment-needed plants recommend `APPLY_PESTICIDE`.
- Gazebo `/scan` is converted into `/dt/physical/environment_state`, so obstacle/environment changes are visible in the digital twin dashboard.

## Repository Layout

```text
CBL-Option-B-Digital-Twin/
  Dockerfile                    ROS 2 Jazzy + TurtleBot3 image (incl. Nav2 ABI fix)
  docker-compose.yml            Cross-platform container setup (VNC GUI path)
  docker-compose.override.yml   Windows/WSL2 only: native GUI via WSLg, no VNC
  docker/                       Small helper scripts inside the container
  my_tb3_world/                 Gazebo arena/world package
  tb3_pesticide_dt/             Mission, Nav2, twin, dashboard, maps, launch files
```

Important launch files:

```text
tb3_pesticide_dt/launch/option_b_full_demo.launch.py    full final demo
tb3_pesticide_dt/launch/pesticide_world.launch.py       Gazebo world only
tb3_pesticide_dt/launch/option_b_navigation2.launch.py  Nav2 only
tb3_pesticide_dt/launch/pesticide_nav2_dt.launch.py     twin/mission nodes only
```

## Cross-Platform Docker Setup

The Docker setup is designed to work on:

- Windows with Docker Desktop and WSL 2
- Linux with Docker Engine
- macOS with Docker Desktop or Colima

There are two ways to see the Gazebo/RViz GUI:

- **Windows + WSL2 (recommended here): native windows via WSLg, no VNC.** The bundled
  `docker-compose.override.yml` (auto-loaded by Compose) binds the WSLg X11 socket so
  Gazebo opens directly on your Windows desktop. Every GUI launch must be prefixed with
  `export DISPLAY=:0` (shown in the commands below).
- **macOS / Linux: VNC.** Remove the override (it is WSL2-only — see *Running on macOS*),
  bring the container up, and view Gazebo through a VNC viewer at `127.0.0.1:5901`
  (password `ros`).

The browser dashboard is served at `http://127.0.0.1:8080` on every platform and needs
no GUI at all — it is the main surface for the demo.

The Compose file uses the `linux/amd64` image platform because the ROS/Gazebo desktop stack is most reliable there. On normal Windows/Linux lab laptops this is native; on Apple Silicon macOS it runs through Docker/Colima emulation.

### 1. Clone The Repo

Windows/WSL or Linux:

```bash
git clone https://github.com/Grasusu/CBL-Autonomous-Twinning-Systems-option-B.git
cd CBL-Autonomous-Twinning-Systems-option-B
```

macOS with Colima:

```bash
colima start --cpu 4 --memory 8 --runtime docker
docker context use colima

git clone https://github.com/Grasusu/CBL-Autonomous-Twinning-Systems-option-B.git
cd CBL-Autonomous-Twinning-Systems-option-B
```

If you are on Windows, the most reliable workflow is to clone inside WSL, not inside `C:\Users\...`, because Linux file mounts are faster and avoid path-sharing issues.

### 2. Build And Start The Container

From the repo root:

```bash
docker compose up --build -d
```

This builds the image `cbl-option-b:jazzy`, starts a container named
`turtlebot3_container`, and mounts the repo at:

```text
/ws/src/cbl_option_b
```

> **Nav2 fix (automatic).** The image upgrades `ros-jazzy-diagnostic-updater` during
> build. Without it the newer Nav2 packages fail to load with
> `undefined symbol: ...diagnostic_updater::Updater...`, `nav2_lifecycle_manager` dies,
> and the robot never navigates. Nothing to do — it is baked into the Dockerfile.
>
> **GUI.** On Windows/WSL2 the bundled `docker-compose.override.yml` renders Gazebo
> natively via WSLg and disables the internal VNC server. On macOS/Linux remove that
> override to fall back to VNC (see *Running on macOS*).

### 3. See The GUI

**Windows + WSL2 (WSLg, no VNC):** nothing to open. The `docker-compose.override.yml`
in this folder binds the WSLg X11 socket, so Gazebo opens as a normal window on your
Windows desktop when you start the demo (Terminal 1 below). Every GUI launch must be
prefixed with `export DISPLAY=:0` so it targets WSLg instead of the disabled internal
display.

**macOS / Linux (VNC):** delete or rename `docker-compose.override.yml` first (it is
WSL2-only), bring the container up, then connect a VNC viewer to:

```text
127.0.0.1:5901      password: ros
```

You will see a small Linux desktop; Gazebo opens there when the demo starts. On this
path do **not** add `export DISPLAY=:0` — the container's default `:1` is the VNC screen.

### 4. Build The ROS Workspace

Run this from a normal host terminal in the repo root:

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install'
```

Quick package check:

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 pkg list | grep -E "my_tb3_world|tb3_pesticide_dt"'
```

Expected output includes:

```text
my_tb3_world
tb3_pesticide_dt
```

## Final Demo Commands

Use two terminals on the host.

### Terminal 1: Full Robot Mission

Windows + WSL2 (native GUI via WSLg — note the `export DISPLAY=:0`):

```bash
docker exec -it turtlebot3_container bash -lc \
'export DISPLAY=:0; cd /ws && ros-env ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=true'
```

macOS / Linux (VNC — no `export DISPLAY=:0`):

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=true'
```

This starts Gazebo, Nav2, AMCL initial pose publishing, the plant mission, and the digital twin nodes. The launch intentionally waits before the robot starts moving so localization and Nav2 are stable. With software OpenGL the Gazebo window may be gray for ~30–60 s before the arena renders — that is normal; wait, and start only one launch at a time.

Expected route:

```text
plant_a -> plant_b -> plant_c -> plant_d -> plant_e -> plant_f -> plant_home
```

### Terminal 2: Browser Dashboard

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 run tb3_pesticide_dt option_b_dashboard_web'
```

Open this on the host:

```text
http://127.0.0.1:8080
```

The dashboard shows:

- physical stand-in state
- digital twin mirrored state
- plant inspection history
- camera health
- environment/obstacle state
- buttons for fault injection and obstacle interaction
- topic evidence for the rubric

## What To Point Out In The Presentation

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

In the dashboard, use:

- `Echo request` for physical to digital
- `Echo result` for digital to physical
- `Show node wiring` for the node/topic view

### 2. State Synchronization

The dashboard mirrors:

```text
mission mode
active plant zone
pose/navigation feedback
camera health
latest inspection result
environment state
```

Use the dashboard buttons:

```text
Inject fault: degraded
Inject fault: failed
Restore: healthy
```

This proves that internal state, not only motion, is synchronized.

### 3. Environmental Interaction

Use:

```text
Drop obstacle on route
Remove obstacle
```

The obstacle is spawned in Gazebo, detected through LIDAR/Nav2, and mirrored in `/dt/physical/environment_state` and the browser dashboard.

## Expected Inspection Results

```text
plant_a -> OK
plant_b -> TREATMENT_NEEDED
plant_c -> OK
plant_d -> TREATMENT_NEEDED
plant_e -> TREATMENT_NEEDED
plant_f -> OK
plant_home -> RETURNED_HOME
```

## Optional Topic Echoes

The browser dashboard already exposes these, but you can echo them manually:

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 topic echo /dt/digital/dashboard_summary std_msgs/msg/String --full-length'
```

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 topic echo /dt/physical/inspection_log std_msgs/msg/String --full-length'
```

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 topic echo /dt/physical/environment_state std_msgs/msg/String --full-length'
```

## Manual Debug Flow

Use this only if the full launch needs debugging.

Terminal 1: Gazebo only

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 launch tb3_pesticide_dt pesticide_world.launch.py gui:=true'
```

Terminal 2: Nav2 only

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 launch tb3_pesticide_dt option_b_navigation2.launch.py \
  use_sim_time:=true \
  map:=/ws/src/cbl_option_b/tb3_pesticide_dt/maps/map.yaml \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_burger_option_b.yaml'
```

Terminal 3: Initial pose

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 run tb3_pesticide_dt nav2_initial_pose_node --ros-args \
  -p use_sim_time:=true \
  -p x:=-0.80 \
  -p y:=-0.07 \
  -p yaw:=0.0 \
  -p duration_s:=14.0'
```

Terminal 4: Mission/twin nodes

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 launch tb3_pesticide_dt pesticide_nav2_dt.launch.py \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_plant_zones.yaml \
  use_sim_time:=true'
```

## Stop Everything

From the repo root:

```bash
docker compose down
```

> `down` removes the container, so the next `docker compose up -d` needs the ROS
> workspace rebuilt again (step 4). To keep the build between sessions, use
> `docker compose stop` / `docker compose start` instead of `down`.

On macOS/Colima, optionally stop Colima too:

```bash
colima stop
```

## Running On macOS / Linux

The `docker-compose.override.yml` shipped here is **Windows/WSL2 only** — it points the
GUI at WSLg, which does not exist on macOS or plain Linux. On those hosts:

1. Remove (or rename) the override so Compose uses only the base file and its VNC server:

   ```bash
   rm docker-compose.override.yml        # or: mv docker-compose.override.yml off.yml.bak
   ```

2. Build and start as usual (`docker compose up --build -d`), then view Gazebo through a
   VNC viewer at `127.0.0.1:5901` (password `ros`).

3. Use the **VNC** form of the Terminal 1 command (without `export DISPLAY=:0`).

Everything else — the Nav2 fix, the workspace build, the mission, and the
`http://127.0.0.1:8080` dashboard — is identical across platforms. On Apple Silicon the
`linux/amd64` image runs under emulation, so expect a slower build and simulation.

## Troubleshooting

If `Package tb3_pesticide_dt not found`, rebuild:

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install'
```

If VNC does not connect:

```bash
docker exec -it turtlebot3_container start-vnc
```

If Gazebo is slow or gets killed, close other heavy apps and run:

```bash
docker compose down
docker compose up -d
```

If Docker says the container does not exist, recreate it:

```bash
docker compose up -d
```

If Windows has mount/path problems, clone the repo inside WSL and run the Docker commands from the WSL terminal.

**Close the demo with `Ctrl-C` in its terminal, not the window's X button.** Closing the
Gazebo window only kills the GUI client; the Gazebo *server* keeps running, and the next
launch shows an empty or stale arena. Run one launch at a time.

If the arena does not show, Gazebo opened and closed, or the container stopped, reset:

```bash
docker restart turtlebot3_container
```

This kills all stray Gazebo/ROS processes but keeps the built workspace; then start one
fresh launch. On Windows/WSL2 also confirm you are launching with `export DISPLAY=:0` and
that `docker-compose.override.yml` is present — without it the container runs the VNC
server, which conflicts with the WSLg socket and exits.

## Verification

Useful checks:

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env python3 -m compileall -q /ws/src/cbl_option_b/tb3_pesticide_dt/tb3_pesticide_dt'
```

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env colcon test --packages-select tb3_pesticide_dt'
```

Headless launch check:

```bash
docker exec -it turtlebot3_container bash -lc \
'cd /ws && ros-env ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=false'
```
