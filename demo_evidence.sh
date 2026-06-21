#!/usr/bin/env bash
# =============================================================================
# demo_evidence.sh  --  Digital Twin demo-evidence helper (Option B)
#
# Argument-driven (no interactive menu, so it is robust inside docker exec).
# The launch + mission must already be running in another terminal.
#
# Usage (run inside the container):
#   bash /ws/src/cbl_option_b/demo_evidence.sh <command>
#
#   topics          Rubric 1: list DT topics + nodes (bidirectional pub/sub)
#   wiring          Rubric 1: node info -> twin & mission subscribe to each other
#   request         Rubric 1: capture one inspection_request (physical -> digital)
#   result          Rubric 1: capture one inspection_result  (digital -> physical)
#   inject-request [zone] [id]  Rubric 1: send a fake request (default plant_f 999);
#                   the twin answers for THAT zone/id -> proves it is listening
#   sync            Rubric 2: physical mission_state vs digital mirror
#   fault X         Rubric 2: inject camera health  X = failed | degraded | healthy
#   env             Rubric 3: capture one environment_state (front_obstacle / min_front)
#   obstacle [x][y] Rubric 3: spawn a NEW obstacle live (default 1.0 -2.80, open bottom leg) -> reroute
#   obstacle-ahead  Rubric 3: auto-aim -> spawn the box ~0.7 m straight in front of the robot
#   obstacle-remove Rubric 3: remove the spawned obstacle
#   pose            Helper: print robot x/y (~gazebo frame) to aim the obstacle
#   snapshot        Save all labeled evidence to /ws/demo_evidence_<time>.log
#   all             Run topics + request + result + sync + env in sequence
#   help            Show this help
# =============================================================================
# NOTE: deliberately NO 'set -u' -- ROS 2 / colcon setup scripts expand unset
# variables and would abort this script the moment it sources them.

PHYS_REQUEST="/dt/physical/inspection_request"
DIG_RESULT="/dt/digital/inspection_result"
PHYS_STATE="/dt/physical/mission_state"
DIG_STATE="/dt/digital/mission_state"
ENV_STATE="/dt/physical/environment_state"
CONTROL="/dt/digital/control"
CAP_TIMEOUT=12

B="\033[1m"; G="\033[1;32m"; Y="\033[1;33m"; C="\033[1;36m"; R="\033[1;31m"; N="\033[0m"

# Source ROS if ros2 is not already on PATH (parent shell usually sourced it).
if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/jazzy/setup.bash 2>/dev/null
  source /opt/turtlebot3_ws/install/setup.bash 2>/dev/null
  [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash 2>/dev/null
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo -e "${R}ERROR:${N} 'ros2' not found. Source ROS first:"
  echo "  source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source /ws/install/setup.bash"
  exit 1
fi

capture() {  # label  topic
  echo -e "${C}== ${1} ==${N}"
  echo -e "  topic: ${B}${2}${N}    (waiting max ${CAP_TIMEOUT}s for one message)\n"
  if ! timeout "${CAP_TIMEOUT}" ros2 topic echo --once "$2"; then
    echo -e "\n  ${Y}No message within ${CAP_TIMEOUT}s.${N} request/result only fire while the robot is AT a plant."
  fi
}

inject() {  # health
  local h="${1:-failed}"
  case "$h" in healthy|degraded|failed) ;; *)
    echo -e "${R}fault expects: healthy | degraded | failed${N}"; exit 2;; esac
  echo -e "${C}== Rubric 2: fault injection (digital -> twin) ==${N}"
  echo -e "  ${CONTROL}  camera_health=${B}${h}${N}\n"
  ros2 topic pub --once "$CONTROL" std_msgs/msg/String "{data: '{\"camera_health\":\"${h}\"}'}"
  echo -e "\n  ${G}Sent.${N} Watch ${DIG_RESULT} / dashboard at the next plant."
}

show_topics() {
  echo -e "${C}== Rubric 1: nodes ==${N}";        ros2 node list
  echo -e "\n${C}== Rubric 1: DT topics (two-way) ==${N}"; ros2 topic list | grep /dt
}

show_wiring() {
  echo -e "${C}== Rubric 1: twin SUBSCRIBES to physical request, PUBLISHES result ==${N}"
  ros2 node info /inspection_twin_node
  echo -e "\n${C}== Rubric 1: mission SUBSCRIBES to digital result, PUBLISHES request ==${N}"
  ros2 node info /plant_nav2_mission_node
}

inject_request() {  # zone  id
  local z="${1:-plant_f}" id="${2:-999}"
  echo -e "${C}== Rubric 1: fake inspection_request (proves the digital twin LISTENS) ==${N}"
  echo -e "  ${PHYS_REQUEST}  zone_id=${B}${z}${N} request_id=${B}${id}${N}\n"
  ros2 topic pub --once "$PHYS_REQUEST" std_msgs/msg/String \
    "{data: '{\"zone_id\":\"${z}\",\"request_id\":${id}}'}"
  echo -e "\n  ${G}Sent.${N} Watch ${DIG_RESULT}: a result appears with zone_id=${z}, request_id=${id}"
  echo -e "  (the twin answered YOUR arbitrary request, not the next scripted zone)."
}

show_sync() {
  echo -e "${C}== Rubric 2: PHYSICAL mission_state ==${N}"
  timeout "${CAP_TIMEOUT}" ros2 topic echo --once "$PHYS_STATE" || echo "(no msg)"
  echo -e "\n${C}== Rubric 2: DIGITAL mission_state (mirror) ==${N}"
  timeout "${CAP_TIMEOUT}" ros2 topic echo --once "$DIG_STATE" || echo "(no msg)"
}

snapshot() {
  local out="/ws/demo_evidence_$(date +%Y%m%d_%H%M%S).log"
  {
    echo "### DIGITAL TWIN EVIDENCE SNAPSHOT  ($(date))"
    echo; echo "===== NODES ====="; ros2 node list
    echo; echo "===== DT TOPICS (bidirectional pub/sub) ====="; ros2 topic list | grep /dt
    echo; echo "===== R1 physical->digital ($PHYS_REQUEST) ====="
    timeout "$CAP_TIMEOUT" ros2 topic echo --once "$PHYS_REQUEST" || echo "(no msg - run at a plant)"
    echo; echo "===== R1 digital->physical ($DIG_RESULT) ====="
    timeout "$CAP_TIMEOUT" ros2 topic echo --once "$DIG_RESULT" || echo "(no msg - run at a plant)"
    echo; echo "===== R2 physical mission_state ($PHYS_STATE) ====="
    timeout "$CAP_TIMEOUT" ros2 topic echo --once "$PHYS_STATE" || echo "(no msg)"
    echo; echo "===== R2 digital mission_state ($DIG_STATE) ====="
    timeout "$CAP_TIMEOUT" ros2 topic echo --once "$DIG_STATE" || echo "(no msg)"
    echo; echo "===== R3 environment_state ($ENV_STATE) ====="
    timeout "$CAP_TIMEOUT" ros2 topic echo --once "$ENV_STATE" || echo "(no msg)"
  } | tee "$out"
  echo -e "\n${G}Saved:${N} ${out}   (on Mac: ~/option_b_ws${out#/ws})"
}

obstacle() {  # x  y   (default = open bottom leg plant_d->plant_e; room to reroute)
  local x="${1:-1.0}" y="${2:--2.80}"
  if ! command -v gz >/dev/null 2>&1; then
    echo -e "${R}gz not found.${N} Run this inside the container where Gazebo is running."; exit 2; fi
cat > /tmp/obstacle.sdf <<'SDF'
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="demo_obstacle">
    <link name="link">
      <collision name="c"><geometry><box><size>0.5 0.5 0.6</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.5 0.5 0.6</size></box></geometry>
        <material><diffuse>0.9 0.2 0.2 1</diffuse></material></visual>
    </link>
  </model>
</sdf>
SDF
  echo -e "${C}== Rubric 3: spawning a NEW obstacle at (${x}, ${y}) ==${N}"
  gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
    --req "sdf_filename: \"/tmp/obstacle.sdf\", pose: {position: {x: ${x}, y: ${y}, z: 0.30}}"
  echo -e "\n  ${G}Spawned.${N} Watch ${ENV_STATE}: front_obstacle -> true, mode -> OBSTACLE_AHEAD; robot reroutes."
}

obstacle_remove() {
  command -v gz >/dev/null 2>&1 || { echo -e "${R}gz not found.${N}"; exit 2; }
  gz service -s /world/default/remove \
    --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
    --req 'name: "demo_obstacle", type: MODEL'
  echo -e "${G}Removed demo_obstacle.${N}"
}

obstacle_ahead() {  # auto-aim: spawn ~0.7 m straight in front of the robot
  command -v gz >/dev/null 2>&1 || { echo -e "${R}gz not found.${N}"; exit 2; }
  echo -e "${C}== Rubric 3: spawning obstacle ~0.7 m in front of the robot ==${N}"
  python3 - <<'PYEOF'
import math, subprocess, sys
try:
    import yaml
except ImportError:
    sys.exit("  PyYAML missing")
try:
    out = subprocess.run(["ros2","topic","echo","--once","/odom"],
                         capture_output=True, text=True, timeout=15).stdout
    doc = yaml.safe_load(out.split("---")[0])
    p = doc["pose"]["pose"]["position"]; o = doc["pose"]["pose"]["orientation"]
except Exception as e:
    sys.exit("  Could not read /odom: %s" % e)
yaw = math.atan2(2*(o["w"]*o["z"]+o["x"]*o["y"]), 1-2*(o["y"]**2+o["z"]**2))
D = 0.7
x = p["x"] + D*math.cos(yaw); y = p["y"] + D*math.sin(yaw)
sdf = ('<?xml version="1.0"?><sdf version="1.8"><model name="demo_obstacle">'
       '<link name="link"><collision name="c"><geometry><box><size>0.5 0.5 0.6</size>'
       '</box></geometry></collision><visual name="v"><geometry><box><size>0.5 0.5 0.6</size>'
       '</box></geometry><material><diffuse>0.9 0.2 0.2 1</diffuse></material></visual>'
       '</link></model></sdf>')
open("/tmp/obstacle.sdf","w").write(sdf)
req = 'sdf_filename: "/tmp/obstacle.sdf", pose: {position: {x: %.3f, y: %.3f, z: 0.30}}' % (x, y)
r = subprocess.run(["gz","service","-s","/world/default/create",
                    "--reqtype","gz.msgs.EntityFactory","--reptype","gz.msgs.Boolean",
                    "--timeout","3000","--req",req], capture_output=True, text=True)
print("  placed at gazebo (%.2f, %.2f), ~0.7 m ahead of the robot" % (x, y))
print("  " + (r.stdout.strip() or r.stderr.strip() or "(no service reply)"))
PYEOF
  echo -e "\n  ${G}Watch ${ENV_STATE}: min_front_m drops; robot reroutes. Then run obstacle-remove.${N}"
}

usage() { sed -n '/^# Usage/,/^# ===/p' "$0" | sed 's/^# \{0,1\}//; s/^==*$//'; }

case "${1:-help}" in
  topics)         show_topics ;;
  wiring)         show_wiring ;;
  request)        capture "Rubric 1: physical -> digital" "$PHYS_REQUEST" ;;
  result)         capture "Rubric 1: digital -> physical" "$DIG_RESULT" ;;
  inject-request) inject_request "$2" "$3" ;;
  sync)           show_sync ;;
  fault)          inject "$2" ;;
  env)            capture "Rubric 3: environment / obstacle" "$ENV_STATE" ;;
  obstacle)       obstacle "$2" "$3" ;;
  obstacle-ahead) obstacle_ahead ;;
  obstacle-remove) obstacle_remove ;;
  pose)           echo -e "${C}== robot x/y (~gazebo frame) ==${N}"
                  ros2 topic echo --once /odom --field pose.pose.position ;;
  snapshot)       snapshot ;;
  all)      show_topics; echo; capture "R1 physical->digital" "$PHYS_REQUEST"; echo
            capture "R1 digital->physical" "$DIG_RESULT"; echo; show_sync; echo
            capture "R3 environment" "$ENV_STATE" ;;
  help|-h|--help) usage ;;
  *) echo -e "${R}Unknown command: $1${N}\n"; usage; exit 2 ;;
esac
