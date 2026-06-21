# TurtleBot3 Plant Health Digital Twin

ROS 2 Jazzy package for the final 2IRR10 Option B digital twin demo.

The official final demo does not use a second visual robot. Gazebo provides one TurtleBot3 as the physical stand-in/source of truth. The digital entity is a ROS 2 control panel plus twin nodes:

```text
option_b_dashboard_node  -> /dt/digital/dashboard, /dt/digital/dashboard_summary, and /dt/digital/control
option_b_dashboard_web   -> browser dashboard at http://127.0.0.1:8080
inspection_twin_node     -> /dt/digital/mission_state and /dt/digital/inspection_result
```

See the repository root `README.md` for the full demo workflow.

## Main Nodes

- `plant_nav2_mission_node`: sends Nav2 goals to 6 plant inspection zones and returns to `plant_home`.
- `inspection_twin_node`: mirrors physical state and simulates hyperspectral plant stress/disease inspection.
- `option_b_environment_node`: converts `/scan` into `/dt/physical/environment_state`.
- `option_b_dashboard_node`: ROS-native digital control panel; publishes full and one-line dashboard state, and sends a camera-health command on `/dt/digital/control`.
- `option_b_dashboard_web`: serves a browser dashboard (http://127.0.0.1:8080) that mirrors state and drives the demo (fault injection, obstacle, topic echoes).

## Final Launches

Preferred full demo:

```bash
ros2 launch tb3_pesticide_dt option_b_full_demo.launch.py gui:=true
```

This starts Gazebo, Nav2, initial localization, the plant mission, and the digital dashboard/control node. It waits about one minute before movement so AMCL/Nav2 are stable.

Manual world:

```bash
ros2 launch tb3_pesticide_dt pesticide_world.launch.py gui:=true
```

Manual Nav2:

```bash
ros2 launch tb3_pesticide_dt option_b_navigation2.launch.py \
  use_sim_time:=true \
  map:=/ws/src/cbl_option_b/tb3_pesticide_dt/maps/map.yaml \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_burger_option_b.yaml
```

Manual mission and digital entity:

```bash
ros2 launch tb3_pesticide_dt pesticide_nav2_dt.launch.py \
  params_file:=/ws/src/cbl_option_b/tb3_pesticide_dt/config/nav2_plant_zones.yaml \
  use_sim_time:=true
```

Dashboard (then open http://127.0.0.1:8080 in a browser):

```bash
ros2 run tb3_pesticide_dt option_b_dashboard_web
```

One-line dashboard summary:

```bash
ros2 topic echo /dt/digital/dashboard_summary std_msgs/msg/String --full-length
```

## Rubric Proof

- Bidirectional pub/sub:
  `/dt/physical/mission_state`, `/dt/physical/inspection_request`, `/dt/physical/environment_state`
  and `/dt/digital/mission_state`, `/dt/digital/inspection_result`, `/dt/digital/control`, `/dt/digital/dashboard`, `/dt/digital/dashboard_summary`.
- State synchronization:
  mission mode, zone, pose, camera health, latest inspection, and final status are mirrored.
- Environmental interaction:
  `/scan` changes are converted to `/dt/physical/environment_state` and reflected in the digital dashboard.

Expected final log:

```text
Plant inspection route complete: RETURNED_HOME
```
