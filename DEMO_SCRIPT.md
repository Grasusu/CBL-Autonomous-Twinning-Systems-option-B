# Demo Script — Option B Digital Twin (all 3 rubrics)

One-page cheat sheet for the recording. For the full Colima/Docker/VNC startup
see DEMO_RUNBOOK.md. Here we assume the demo is already launched and the robot
is moving (mission starts ~65 s after launch).

## Terminals
- **Gazebo window** (VNC) — to see the robot move/reroute.
- **DASH**: `ros2 run tb3_pesticide_dt option_b_dashboard_viewer` — keep visible the whole time.
- A few echo terminals + one **CTRL** terminal for commands.

In every terminal first run:
```bash
source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source /ws/install/setup.bash
D=/ws/src/cbl_option_b/demo_evidence.sh    # short alias for the helper
```

The DASH panel shows everything at a glance, including the line:
`rubric: bidirectional=.. | state_sync=.. | environment=..`

================================================================
## RUBRIC 1 — Bidirectional pub/sub
================================================================
**Idea:** physical publishes a request the digital reads; digital publishes a
result the physical uses. Two-way.

**Echo terminals:**
```bash
ros2 topic echo /dt/physical/inspection_request   # physical -> digital
ros2 topic echo /dt/digital/inspection_result     # digital -> physical
```
**Main proof (wiring):**
```bash
bash $D wiring     # node info: each node subscribes to the other's topic
```
SAY: *"When the robot reaches a plant, the physical side publishes an inspection
request; the digital twin answers with a result, and the physical side records it.
`node info` shows each node is subscribed to the other's topic — wired together,
both directions."*

**Skeptic backup (only if asked):**
```bash
bash $D inject-request    # fake request plant_f/999 -> twin answers for THAT zone/id
bash $D fault failed      # physical log shows SENSOR_FAILED (exists in no config)
bash $D fault healthy
```
SAY: *"I send a request for an arbitrary zone with a unique id — the twin answers
for exactly that one, so it's reading my message. And `failed` makes the physical
side report SENSOR_FAILED, a state no config produces — so it genuinely uses the
digital result, not a script."*

================================================================
## RUBRIC 2 — State synchronization
================================================================
**Idea:** operational STATE (not just commands) syncs both ways:
mission mode (physical->digital) and camera health (digital->physical).

**Echo terminals:**
```bash
ros2 topic echo /dt/physical/mission_state    # field: mode, digital_camera_health
ros2 topic echo /dt/digital/mission_state     # field: mode (mirror), camera_health
```
**Step 1 (mode mirror, passive):** as the robot runs, `mode` in both goes
NAVIGATING -> INSPECTING together (also visible on DASH).

**Step 2 (camera health + behavior change):**
```bash
bash $D fault degraded     # digital camera_health=degraded
```
- digital state `camera_health=degraded`; physical state `digital_camera_health=degraded`
- next plant: inspection `confidence` drops 0.93 -> 0.68
```bash
bash $D fault healthy      # restore
```
SAY: *"Two states are synchronized. The mission mode mirrors physical->digital live.
I set the camera health on the digital side and it appears on the physical side and
changes the inspection confidence — state synchronization that affects behavior."*

**Skeptic backup:**
```bash
bash $D fault failed       # physical reports SENSOR_FAILED (config-impossible)
bash $D fault healthy
```

================================================================
## RUBRIC 3 — Environmental interaction
================================================================
**Idea:** an environment event propagates through the twin and changes planning.

**Echo terminal:**
```bash
ros2 topic echo /dt/physical/environment_state   # front_obstacle, min_front_m, environment_mode
```
**Step 1:** baseline — `environment_mode=CLEAR`, `front_obstacle=false`.

**Step 2 (introduce obstacle live):** while the robot is on the bottom leg (plant_d->plant_e):
```bash
bash $D obstacle           # spawns a NEW box at (1.0, -2.80), open leg with room to reroute
```
(Avoid the top leg near x=0.77 — there is already an arena obstacle there; a box on top
of it blocks the path and the robot will skip the plant instead of rerouting.)
- env state -> `front_obstacle=true`, `environment_mode=OBSTACLE_AHEAD`, `min_front_m` drops
- Gazebo: the robot reroutes around it
```bash
bash $D obstacle-remove    # remove after the moment
```
SAY: *"I drop a new obstacle during the run. The robot's LIDAR detects it, the
environment state propagates to the digital entity as OBSTACLE_AHEAD, and Nav2
replans around it. The box is in no world file and `min_front_m` is a live distance —
so it's real sensing propagating through the twin and changing planning."*

================================================================
## CLOSING
================================================================
Point at the DASH rubric line: `bidirectional=OK | state_sync=OK | environment=OK`.
SAY: *"All three digital-twin usages demonstrated live."*

Save the recorded evidence:
```bash
cp /tmp/tb3_option_b_demo_evidence.jsonl /ws/    # -> Mac: ~/option_b_ws/...jsonl
```

## Recording order (one clean run)
1. Launch + DASH; wait ~65 s for the robot to move.
2. Rubric 1: echoes at a plant + `wiring`.
3. Rubric 2: mode mirror + `fault degraded` -> `fault healthy`.
4. Rubric 3: `obstacle` -> reroute -> `obstacle-remove`.
5. Let it finish: `final=RETURNED_HOME`, `progress=6/6`.
6. Show DASH rubric flags all OK; save JSONL.

Timing: `fault ...` and `obstacle` apply to the NEXT plant/leg — run them just
before the robot gets there.
