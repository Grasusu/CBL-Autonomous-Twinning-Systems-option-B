# One-take demo — web dashboard only

Follow this top to bottom while recording. You only need **two windows**: the
**browser** at http://127.0.0.1:8080 and the **Gazebo (VNC)** window. Cue each
action off the dashboard's **`zone`** field (top of the Physical card) — that is
more reliable than a stopwatch when the sim runs slow.

Architecture in one line: the **robot carries the (simulated) camera** — it drives
to each plant, turns to face it, and sends its reading to the **digital twin**,
which does the analysis. LIDAR (obstacle detection) is a real sensor on the robot.

The route is fixed: `plant_a → plant_b → plant_c → plant_d → plant_e → plant_f → home`.
Expected results (so you can narrate ahead): a OK, **b TREAT**, c OK, **d TREAT**,
**e TREAT**, f OK.

---

## 0. Before you hit record
- Launch is up and the dashboard says **live** (green pill, top-left).
- Robot has started moving (zone shows `plant_a`/`plant_b`). The first ~65 s after
  launch the robot is still — wait for it before recording.

## 1. Intro (zone = plant_a) — ~20 s
Point at the layout and say it out loud:
- **Physical card** = the Gazebo robot + mission node, **and the camera** (the
  "physical entity").
- **Digital twin card** = the inspection twin that analyses the readings (the
  "digital entity").
- Note the robot **turns to face each plant** before inspecting — that's the camera
  capturing.

## 2. Rubric 1 — bidirectional pub/sub (zone = plant_b)
1. Click **Show node wiring**. Read it: `plant_nav2_mission_node` *publishes* the
   reading and *subscribes* to the result; `inspection_twin_node` does the mirror.
   Each subscribes to the other → wired both ways.
2. Click **Echo request (R1 phys→dig)** → the **camera reading the robot captured**
   (`measured_plant_stress`, `camera_health`).
3. Click **Echo result (R1 dig→phys)** → the **twin's analysis** of that reading
   (status, recommendation, confidence).
4. Say: *"The robot's camera sends the reading, the twin computes the result and
   sends it back — two-way."*

## 3. Rubric 2 — state synchronization (zone = plant_c → plant_d)
1. Passive proof: point at **Physical card `mode`** and **Digital card
   `mode (mirrored)`** — they change together (NAVIGATING → INSPECTING) as the robot
   works. State, not a command, syncing physical → digital.
2. Click **Inject fault: degraded** (an operator command to the robot's camera).
   - Physical card **camera health (robot)** → amber **DEGRADED**.
   - Digital card **camera health** → **DEGRADED** at the same time → *"the robot's
     camera state is mirrored on the digital twin."*
   - (Optional) **Echo physical state** / **Echo digital state** to show the same
     `degraded` value on both `/dt/physical/mission_state` and `/dt/digital/mission_state`.
3. Watch the next plant's row: **confidence drops** (0.93 → 0.68) — the degraded
   camera changes the recorded result.
4. Click **Restore: healthy**.

## 4. Rubric 3 — environmental interaction (right after plant_d, heading to plant_e)
> The box drops at a **fixed point** on the plant_d → plant_e leg. Click **right
> after plant_d is inspected** (zone still shows `plant_d` / just switched toward
> `plant_e`) so the robot then drives into it. Clicking too late means the robot has
> already passed the spot.
1. Click **Drop obstacle on route**.
   - **Environment card**: mode → **OBSTACLE_AHEAD**, front obstacle → red **YES**,
     min front drops to a live distance.
   - **Gazebo window**: the red box appears and the robot **reroutes** around it.
2. Click **Echo environment (R3)** → show `front_obstacle: true` in the message.
3. Say: *"The box isn't in the map — the robot's LIDAR detects it, the environment
   state propagates to the digital side, and Nav2 replans."*
4. Click **Remove obstacle**.

## 5. Finish (zone = plant_f → home)
- Let it inspect `plant_f` and drive home.
- Progress reaches **6 / 6**, the `final` tag shows **HOME** (`RETURNED_HOME`).
- Point at the results table + the live cards: *"All three digital-twin usages,
  demonstrated live, from one dashboard."*

---

## If the robot is crawling or stops mid-leg
The sim's real-time factor drops when the Mac/Colima CPU is saturated — the robot
then *looks* slow, and Nav2 briefly **stops to run a recovery / replan** when it
can't keep its 10 Hz control loop. Watch the **`recoveries`** counter on the
Physical card: if it ticks up at each pause, that is what is happening (not a bug in
the twin). To help:
- Run **only one** dashboard (web *or* terminal viewer) and close extra terminals.
- Give Colima more cores at start: `colima start --cpu 6 --memory 12 ...`.
- Don't spam **Show node wiring** — it spawns a `ros2` process each click. The Echo
  buttons are cheap (cached), use those freely.
- The old always-on world obstacle is gone, so the plant_d→plant_e leg is clear
  until you drop the box yourself.
