# Demo Evidence Checklist

Use this during the final demo recording. Start the evidence terminal before
launching the mission so every inspection log is captured.

## 1. Bidirectional Pub/Sub

Show the digital twin topics:

```bash
ros2 topic list | grep /dt
```

Physical/source side to digital side:

```bash
ros2 topic echo /dt/physical/mission_state
ros2 topic echo /dt/physical/inspection_request
```

Digital side to physical/source side:

```bash
ros2 topic echo /dt/digital/mission_state
ros2 topic echo /dt/digital/inspection_result
```

## 2. State Synchronization

Show that mission state is mirrored:

```bash
ros2 topic echo /dt/physical/mission_state
ros2 topic echo /dt/digital/mission_state
```

Optional live state-change demo:

```bash
ros2 param set /inspection_twin_node camera_health degraded
ros2 topic echo /dt/digital/mission_state
ros2 topic echo /dt/physical/mission_state
```

Set it back after showing the state:

```bash
ros2 param set /inspection_twin_node camera_health healthy
```

## 3. Environmental Interaction

Show the robot reaching plant zones, waiting, and receiving inspection results:

```bash
ros2 topic echo /dt/physical/inspection_log std_msgs/msg/String --full-length
```

The final mission should include these lines in the mission terminal:

```text
Route waypoint 9 is plant_home
Sent Nav2 return goal plant_home: Home / Start
Plant inspection route complete: RETURNED_HOME
```

For the safety part inherited from the scanner mini-project, use the hybrid
fallback or safety-node launch and show:

```bash
ros2 topic echo /dt/safety_state
```
