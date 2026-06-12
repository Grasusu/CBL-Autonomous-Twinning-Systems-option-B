# TurtleBot3 Plant Health Digital Twin

ROS 2 Jazzy package for the final 2IRR10 Option B digital twin demo.

The official final demo does not use a second visual robot. Gazebo provides one TurtleBot3 as the physical stand-in/source of truth. The digital entity is a ROS 2 control panel plus twin nodes:

```text
option_b_dashboard_node  -> /dt/digital/dashboard, /dt/digital/dashboard_summary, and /dt/digital/control
option_b_dashboard_viewer -> readable terminal dashboard for the demo
inspection_twin_node     -> /dt/digital/mission_state and /dt/digital/inspection_result
demo_evidence_node       -> /dt/demo_evidence rubric summary
evidence_recorder_node   -> /dt/evidence_recording and JSONL proof file
```

The main runbook is the repository root `README.md`.

## Main Nodes

- `plant_nav2_mission_node`: sends Nav2 goals to 6 plant inspection zones and returns to `plant_home`.
- `inspection_twin_node`: mirrors physical state and simulates hyperspectral plant stress/disease inspection.
- `option_b_environment_node`: converts `/scan` into `/dt/physical/environment_state`.
- `option_b_dashboard_node`: ROS-native digital control panel; publishes full and one-line dashboard state, and sends a camera-health command on `/dt/digital/control`.
- `option_b_dashboard_viewer`: subscribes to `/dt/digital/dashboard` and prints a compact readable live dashboard.
- `demo_evidence_node`: aggregates bidirectional communication, state synchronization, and environmental interaction evidence.
- `evidence_recorder_node`: records proof topics to `/tmp/tb3_option_b_demo_evidence.jsonl`.

## Final Launches

World:

```bash
ros2 launch tb3_pesticide_dt pesticide_world.launch.py gui:=true
```

Nav2:

```bash
ros2 launch tb3_pesticide_dt option_b_navigation2.launch.py \
  use_sim_time:=true \
  map:=/ws/src/cbl_option_b/tb3_pesticide_dt/maps/map.yaml \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_burger_option_b.yaml
```

Mission and digital entity:

```bash
ros2 launch tb3_pesticide_dt pesticide_nav2_dt.launch.py \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_plant_zones.yaml \
  use_sim_time:=true
```

Dashboard:

```bash
ros2 run tb3_pesticide_dt option_b_dashboard_viewer
```

One-line dashboard evidence:

```bash
ros2 topic echo /dt/digital/dashboard_summary std_msgs/msg/String --full-length
```

Evidence:

```bash
ros2 topic echo /dt/evidence_recording std_msgs/msg/String --full-length
ros2 topic echo /dt/demo_evidence std_msgs/msg/String --full-length
```

## Rubric Proof

- Bidirectional pub/sub:
  `/dt/physical/mission_state`, `/dt/physical/inspection_request`, `/dt/physical/environment_state`
  and `/dt/digital/mission_state`, `/dt/digital/inspection_result`, `/dt/digital/control`, `/dt/digital/dashboard`, `/dt/digital/dashboard_summary`.
- State synchronization:
  mission mode, zone, pose, camera health, latest inspection, and final status are mirrored.
- Environmental interaction:
  `/scan` changes are converted to `/dt/physical/environment_state` and reflected in digital dashboard/evidence.

Expected final log:

```text
Plant inspection route complete: RETURNED_HOME
```
