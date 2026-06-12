# Demo Evidence Checklist

Use this during the final demo recording. Start the evidence terminal before
launching the mission so every inspection log is captured.

The easiest final recording flow is:

1. Start Gazebo, Nav2, the mission launch, and the evidence/dashboard terminal.
2. Start the mission.
3. Keep `/dt/evidence_recording` or `/dt/demo_evidence` visible while the robot moves.
4. After the first inspection result, optionally inject `camera_health=degraded` to show a live digital-to-physical state change.
5. At the end, show the final `RETURNED_HOME` log and the recorded JSONL file.

## 1. Bidirectional Pub/Sub

Show the digital twin topics:

```bash
ros2 topic list | grep /dt
ros2 run tb3_pesticide_dt option_b_dashboard_viewer
ros2 topic echo /dt/digital/dashboard_summary std_msgs/msg/String --full-length
ros2 topic echo /dt/demo_evidence std_msgs/msg/String --full-length
ros2 topic echo /dt/evidence_recording std_msgs/msg/String --full-length
```

Physical/source side to digital side:

```bash
ros2 topic echo /dt/physical/mission_state
ros2 topic echo /dt/physical/inspection_request
ros2 topic echo /dt/physical/environment_state
```

Digital side to physical/source side:

```bash
ros2 topic echo /dt/digital/mission_state
ros2 topic echo /dt/digital/inspection_result
ros2 topic echo /dt/digital/dashboard_summary std_msgs/msg/String --full-length
ros2 topic echo /dt/digital/dashboard std_msgs/msg/String --full-length
ros2 topic pub --once /dt/digital/control std_msgs/msg/String \
  "{data: '{\"camera_health\":\"degraded\"}'}"
```

## 2. State Synchronization

Show that mission state is mirrored:

```bash
ros2 topic echo /dt/physical/mission_state
ros2 topic echo /dt/digital/mission_state
```

Optional live state-change demo:

```bash
ros2 topic pub --once /dt/digital/control std_msgs/msg/String \
  "{data: '{\"camera_health\":\"degraded\"}'}"
ros2 topic echo /dt/digital/mission_state
ros2 topic echo /dt/physical/mission_state
```

Set it back after showing the state:

```bash
ros2 topic pub --once /dt/digital/control std_msgs/msg/String \
  "{data: '{\"camera_health\":\"healthy\"}'}"
```

## 3. Environmental Interaction

Show the robot reaching plant zones, waiting, and receiving inspection results:

```bash
ros2 topic echo /dt/physical/environment_state std_msgs/msg/String --full-length
ros2 topic echo /dt/physical/inspection_log std_msgs/msg/String --full-length
```

In `/dt/demo_evidence`, point at:

```text
environmental_interaction.environment_change_seen
environmental_interaction.front_obstacle_seen
environmental_interaction.min_front_seen_m
environmental_interaction.max_front_seen_m
```

`environmental_interaction` should become ready only after the scan values change during motion or an obstacle is seen in front of the robot.

## 4. Automatic Recorded Evidence

The Option B mission launch starts `evidence_recorder_node`. It writes all raw proof messages to:

```bash
/tmp/tb3_option_b_demo_evidence.jsonl
```

Show the latest records after the route:

```bash
tail -n 80 /tmp/tb3_option_b_demo_evidence.jsonl
```

This is the easiest way to prove that `/dt/demo_evidence` and `/dt/digital/dashboard` are dashboards over real ROS topic traffic.

The final mission should include these lines in the mission terminal:

```text
Route waypoint 7 is plant_home
Sent Nav2 return goal plant_home: Home / Start
Plant inspection route complete: RETURNED_HOME
```

For the safety part inherited from the scanner mini-project, use the hybrid
fallback or safety-node launch and show:

```bash
ros2 topic echo /dt/safety_state
```
