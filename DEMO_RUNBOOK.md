# Option B — Demo Runbook (step-by-step)

Full sequence to run the plant-inspection digital-twin demo from a cold Mac,
record it, and show evidence for the 3 rubric items.

You will open **6 terminals total**: 2 on the Mac (Colima/Docker + VNC),
4 inside the container (launch, dashboard, evidence, controls).

---

## Step 0 — One-time check
- Colima installed (`colima version`), Docker CLI, a VNC viewer (e.g. RealVNC / macOS Screen Sharing).
- The Docker image `turtlebot3_ws_vnc` exists (`docker images | grep turtlebot3_ws_vnc`).

---

## Step 1 — Terminal 1 (Mac): Colima + sync source + start container

```bash
colima start --cpu 4 --memory 8 --runtime docker
docker context use colima

mkdir -p "$HOME/option_b_ws/src"

# push the latest source into the workspace the container mounts
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

> `-p 8080:8080` is only needed for the **web** dashboard (Step 4, option B). The
> terminal dashboard needs no extra port. If you forgot it, `docker rm -f
> turtlebot3_container` and re-run with the extra `-p`.

Leave this terminal open (it holds the container).

---

## Step 2 — Terminal 2 (Mac): start VNC, then connect

```bash
docker exec turtlebot3_container bash -lc \
'rm -f /tmp/.X1-lock /tmp/.X11-unix/X1; vncserver -kill :1 2>/dev/null || true; vncserver :1 -geometry 1280x800 -depth 24 -localhost no -rfbport 5901'
```

Open your VNC viewer at **`127.0.0.1:5901`**, password **`ros`**.
(The Gazebo window will appear here once you launch.)

---

## Step 3 — Terminal 3 (container): build + launch the demo

```bash
docker exec -it turtlebot3_container bash
```
Then inside:
```bash
cd /ws

# clean any old sim so the new world reloads
pkill -f "ros2 launch" || true
pkill -f "gz sim" || true
pkill -f "component_container" || true
sleep 3

source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build --packages-select my_tb3_world tb3_pesticide_dt --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:1

ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=true
```

IMPORTANT — timing: the mission starts ~65 s after launch (Nav2 warm-up timers).
The robot does NOT move for the first minute — this is normal. Wait for it.

This single launch auto-starts everything: Gazebo + robot, Nav2, inspection
twin, environment node, demo_evidence_node, evidence_recorder_node,
dashboard_node, mission node.

---

## Step 4 — Terminal 4 (container): readable digital twin (the star of the video)

Pick **one** (both read the same `/dt/digital/dashboard` topic, so you can run
either or both — they don't interfere).

### Option A — terminal dashboard (no extra port needed)
```bash
docker exec -it turtlebot3_container bash
```
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 run tb3_pesticide_dt option_b_dashboard_viewer
```
Live panel shows: `progress=X/6`, mission mode, camera health, latest plant
result, env obstacle, and the three rubric flags. Keep on screen the whole time.

### Option B — web dashboard (needs `-p 8080:8080` from Step 1)
```bash
docker exec -it turtlebot3_container bash
```
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 run tb3_pesticide_dt option_b_dashboard_web
```
Then open **http://127.0.0.1:8080** in a browser on your Mac. Same data as the
terminal panel, plus colour-coded rubric badges **and buttons that drive the
entire demo from the page** — so you can skip Terminals 5 & 6:

- **Rubric 1 & 2** — `Inject fault: degraded / failed / healthy` publishes
  `/dt/digital/control`; the camera health then re-appears on the **physical**
  card as `digital_camera_health` and changes the next plant's confidence.
  `Inject fake request (plant_f)` publishes `/dt/physical/inspection_request`
  with a fresh id and shows the twin's actual reply for **that id** inline.
- **Rubric 3** — `Drop obstacle on route` spawns a box at the fixed gazebo-world
  spot on the open bottom leg (1.0, -2.80) via `gz service` (best-effort removes
  any previous one first), and `Remove obstacle` clears it. Press it while the
  robot is heading down that leg.
- **Rubric evidence** — `Echo request / result / physical state / digital state /
  environment` print the latest message on each topic, and `Show node wiring`
  runs `ros2 node info` for both nodes.

Options: `--ros-args -p web_port:=9090` (change port) or
`-p enable_control:=false` (view-only, hides all action buttons).

---

## Step 5 — Terminal 5 (container): evidence stream

```bash
docker exec -it turtlebot3_container bash
```
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 topic echo /dt/demo_evidence
```
Shows `rubric_ready` flags, `message_counts` (both directions), `fresh_topics`.

---

## Step 6 — Terminal 6 (container): the two interactive demo events

```bash
docker exec -it turtlebot3_container bash
```
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
```

### 6a — State synchronization (camera-health fault)
```bash
bash /ws/src/cbl_option_b/demo_evidence.sh fault degraded
# watch dashboard camera -> degraded, then restore:
bash /ws/src/cbl_option_b/demo_evidence.sh fault healthy
```

### 6b — Environmental interaction (drop an obstacle live, while robot moves)
```bash
cat > /tmp/obstacle.sdf << 'EOF'
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="demo_obstacle">
    <link name="link">
      <collision name="c"><geometry><box><size>0.3 0.3 0.5</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.3 0.3 0.5</size></box></geometry>
        <material><diffuse>0.9 0.2 0.2 1</diffuse></material></visual>
    </link>
  </model>
</sdf>
EOF

# drop it on the OPEN bottom leg (plant_d -> plant_e), in front of the robot.
# Avoid the top leg near x=0.77 (an arena obstacle is there -> robot would skip the plant).
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'sdf_filename: "/tmp/obstacle.sdf", pose: {position: {x: 1.0, y: -2.80, z: 0.25}}'

# remove after the moment:
gz service -s /world/default/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "demo_obstacle", type: MODEL'
```

---

## Step 7 — Save the recorded proof (after the run)
```bash
cp /tmp/tb3_option_b_demo_evidence.jsonl /ws/
# on the Mac it appears at ~/option_b_ws/tb3_option_b_demo_evidence.jsonl
```

---

## Recording order (one clean run)
1. Start launch (T3); wait ~65 s for the robot to start moving.
2. Show dashboard (T4) — progress climbs, modes change.
3. Show evidence stream (T5) — counts both directions, rubric flags.
4. Drop an obstacle (T6b) — env flips to OBSTACLE_AHEAD, robot reroutes.
5. Inject camera fault (T6a) — camera health mirrors, inspection confidence drops.
6. Let it finish — `final=RETURNED_HOME`, progress `6/6`.
7. Save JSONL (Step 7).

## Quick troubleshooting
- Gazebo window blank / frozen: re-run Step 2 (VNC), reconnect.
- Robot never moves: confirm you waited ~65 s; check Nav2 came up in T3 logs.
- `request`/`result` show "no message": only fire while the robot is AT a plant.
- Obstacle ignored: place it closer (< 0.45 m) directly in front; box must be >= 0.2 m tall.
- Plants don't block the robot: expected — plant markers have no collision; use a spawned box.
