#!/usr/bin/env python3
"""Browser dashboard for the Option B digital twin.

A ROS node that subscribes to the consolidated ``/dt/digital/dashboard`` JSON
topic and serves a single self-contained HTML page, streaming live updates to
the browser via Server-Sent Events (SSE).

On top of *viewing*, the page also drives the demo actions, so the whole
presentation can be given from the browser alone:

  * Rubric 1/2  -- inject a camera fault          -> /dt/digital/control (operator
                   command picked up by the robot's camera; the twin mirrors the health)
  * Rubric 1    -- show node wiring / topic echo  -> ros2 node info + cached topic msgs
  * Rubric 3    -- drop / remove an obstacle      -> gz service create / remove at a
                   fixed gazebo-world coordinate on the plant_d -> plant_e leg

Only the Python standard library is used for the web side (``http.server`` +
SSE), so there are no extra pip/apt dependencies. The obstacle and wiring buttons
shell out to the ``gz`` / ``ros2`` CLIs already on PATH inside the container.
"""

import json
import os
import queue
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


OBSTACLE_SDF_PATH = "/tmp/obstacle.sdf"


def build_obstacle_sdf(sx: float, sy: float, sz: float) -> str:
    """A red box of the given x/y/z size, named demo_obstacle."""
    size = f"{sx} {sy} {sz}"
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.8"><model name="demo_obstacle">'
        f'<link name="link"><collision name="c"><geometry><box><size>{size}</size></box>'
        '</geometry></collision><visual name="v"><geometry><box>'
        f'<size>{size}</size></box></geometry>'
        '<material><ambient>0.7 0.15 0.15 1</ambient><diffuse>0.9 0.2 0.2 1</diffuse></material>'
        '</visual></link></model></sdf>'
    )

# key -> (parameter name, default topic) for the live "echo" buttons.
ECHO_TOPICS = {
    "request": ("inspection_request_topic", "/dt/physical/inspection_request"),
    "result": ("inspection_result_topic", "/dt/digital/inspection_result"),
    "physical_state": ("physical_state_topic", "/dt/physical/mission_state"),
    "digital_state": ("digital_state_topic", "/dt/digital/mission_state"),
    "environment_state": ("environment_state_topic", "/dt/physical/environment_state"),
}


# --------------------------------------------------------------------------- #
# The single-page web app (no external assets, works fully offline).
# --------------------------------------------------------------------------- #
INDEX_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>SMARTLE - Option B Digital Twin</title>
<style>
  :root {
    --bg: #0d1117; --panel: #161b22; --panel2: #1c2330; --border: #30363d;
    --text: #e6edf3; --muted: #8b949e; --accent: #2f81f7;
    --ok: #3fb950; --warn: #d29922; --bad: #f85149; --treat: #db6d28;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--text);
    font-family: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace;
    font-size: 14px; line-height: 1.45;
  }
  header {
    display: flex; align-items: center; gap: 16px; flex-wrap: wrap;
    padding: 14px 20px; border-bottom: 1px solid var(--border); background: var(--panel);
  }
  header h1 { font-size: 18px; margin: 0; letter-spacing: 1px; }
  .pill {
    font-size: 12px; padding: 3px 9px; border-radius: 999px;
    border: 1px solid var(--border); color: var(--muted);
  }
  .pill.live { color: var(--ok); border-color: var(--ok); }
  .pill.stale { color: var(--bad); border-color: var(--bad); }
  .wrap { padding: 18px 20px; max-width: 1100px; margin: 0 auto; }
  .progress-row { display: flex; align-items: center; gap: 12px; margin-bottom: 18px; }
  .bar { flex: 1; height: 14px; background: var(--panel2); border-radius: 7px; overflow: hidden; border: 1px solid var(--border); }
  .bar > div { height: 100%; width: 0; background: linear-gradient(90deg, var(--accent), var(--ok)); transition: width .4s ease; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 14px; }
  .card { background: var(--panel); border: 1px solid var(--border); border-radius: 10px; padding: 14px 16px; }
  .card h2 { font-size: 12px; text-transform: uppercase; letter-spacing: 1px; color: var(--muted); margin: 0 0 10px; }
  .kv { display: flex; justify-content: space-between; gap: 10px; padding: 3px 0; }
  .kv .k { color: var(--muted); }
  .kv .v { text-align: right; font-weight: 600; }
  table { width: 100%; border-collapse: collapse; margin-top: 4px; }
  th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid var(--border); }
  th { color: var(--muted); font-weight: 600; font-size: 12px; text-transform: uppercase; }
  .tag { display: inline-block; padding: 1px 8px; border-radius: 6px; font-size: 12px; font-weight: 700; }
  .tag.ok { background: rgba(63,185,80,.15); color: var(--ok); }
  .tag.treat { background: rgba(219,109,40,.18); color: var(--treat); }
  .tag.bad { background: rgba(248,81,73,.15); color: var(--bad); }
  .tag.warn { background: rgba(210,153,34,.15); color: var(--warn); }
  .tag.muted { background: rgba(139,148,158,.12); color: var(--muted); }
  .rubric { display: flex; gap: 12px; flex-wrap: wrap; margin-top: 4px; }
  .badge { flex: 1; min-width: 150px; border: 1px solid var(--border); border-radius: 10px; padding: 12px 14px; background: var(--panel2); }
  .badge .label { font-size: 11px; text-transform: uppercase; letter-spacing: 1px; color: var(--muted); }
  .badge .state { font-size: 18px; font-weight: 800; margin-top: 4px; }
  .badge.on { border-color: var(--ok); }
  .badge.on .state { color: var(--ok); }
  .badge.off .state { color: var(--warn); }
  .counts { display: flex; gap: 18px; flex-wrap: wrap; color: var(--muted); font-size: 13px; margin-top: 8px; }
  .counts b { color: var(--text); }
  .controls { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 6px; }
  button {
    font: inherit; cursor: pointer; border-radius: 8px; padding: 9px 14px;
    border: 1px solid var(--border); background: var(--panel2); color: var(--text);
  }
  button:hover { border-color: var(--accent); }
  button.bad:hover { border-color: var(--bad); }
  button.ok:hover { border-color: var(--ok); }
  .note { color: var(--muted); font-size: 12px; margin-top: 8px; }
  pre { background: var(--bg); border: 1px solid var(--border); border-radius: 8px; padding: 12px;
        max-height: 340px; overflow: auto; white-space: pre-wrap; font-size: 12px; color: #c9d1d9; margin: 10px 0 0; }
  section { margin-top: 16px; }
</style>
</head>
<body>
  <header>
    <h1>SMARTLE &middot; OPTION B DIGITAL TWIN</h1>
    <span id="link" class="pill stale">connecting...</span>
    <span id="age" class="pill">updated -s</span>
    <span id="uptime" class="pill">uptime -s</span>
  </header>

  <div class="wrap">
    <div class="progress-row">
      <strong id="progress-label">0 / 6 plants</strong>
      <div class="bar"><div id="progress-bar"></div></div>
      <span id="final" class="tag muted">-</span>
    </div>

    <div class="grid">
      <div class="card">
        <h2>Physical (stand-in)</h2>
        <div class="kv"><span class="k">mode</span><span class="v" id="p-mode">-</span></div>
        <div class="kv"><span class="k">zone</span><span class="v" id="p-zone">-</span></div>
        <div class="kv"><span class="k">pose x / y</span><span class="v" id="p-pose">-</span></div>
        <div class="kv"><span class="k">yaw</span><span class="v" id="p-yaw">-</span></div>
        <div class="kv"><span class="k">nav remaining</span><span class="v" id="p-nav">-</span></div>
        <div class="kv"><span class="k">recoveries</span><span class="v" id="p-recov">-</span></div>
        <div class="kv"><span class="k">camera health (robot)</span><span class="v" id="p-dch">-</span></div>
      </div>

      <div class="card">
        <h2>Digital twin (mirror)</h2>
        <div class="kv"><span class="k">mode (mirrored)</span><span class="v" id="d-mode">-</span></div>
        <div class="kv"><span class="k">mirrored zone</span><span class="v" id="d-zone">-</span></div>
        <div class="kv"><span class="k">camera health</span><span class="v" id="d-cam">-</span></div>
        <div class="kv"><span class="k">control cmd</span><span class="v" id="d-ctrl">-</span></div>
        <div class="kv"><span class="k">latest result</span><span class="v" id="d-latest">-</span></div>
      </div>

      <div class="card">
        <h2>Environment</h2>
        <div class="kv"><span class="k">mode</span><span class="v" id="e-mode">-</span></div>
        <div class="kv"><span class="k">front obstacle</span><span class="v" id="e-obst">-</span></div>
        <div class="kv"><span class="k">min front</span><span class="v" id="e-front">-</span></div>
        <div class="kv"><span class="k">scan stale</span><span class="v" id="e-stale">-</span></div>
      </div>
    </div>

    <section>
      <div class="card">
        <h2>Inspection results</h2>
        <table>
          <thead><tr><th>Plant</th><th>Result</th><th>Stress</th><th>Action</th><th>Camera</th></tr></thead>
          <tbody id="results"><tr><td colspan="5" class="note">waiting for first plant result...</td></tr></tbody>
        </table>
      </div>
    </section>

    <div id="control-section" style="display:none">
      <section>
        <div class="card">
          <h2>Rubric 1 &amp; 2 &middot; Inject camera fault (operator command &rarr; robot camera)</h2>
          <div class="controls">
            <button class="bad" onclick="sendControl('degraded')">Inject fault: degraded</button>
            <button class="bad" onclick="sendControl('failed')">Inject fault: failed</button>
            <button class="ok" onclick="sendControl('healthy')">Restore: healthy</button>
          </div>
          <div class="note" id="control-msg"></div>
        </div>
      </section>

      <section>
        <div class="card">
          <h2>Rubric 3 &middot; Environmental interaction</h2>
          <div class="controls">
            <button class="bad" onclick="obstacle('drop')">Drop obstacle on route</button>
            <button class="ok" onclick="obstacle('remove')">Remove obstacle</button>
          </div>
          <div class="note" id="obstacle-msg"></div>
        </div>
      </section>

      <section>
        <div class="card">
          <h2>Rubric evidence &middot; live topic echoes</h2>
          <div class="controls">
            <button onclick="echo('request')">Echo request (R1 phys&rarr;dig)</button>
            <button onclick="echo('result')">Echo result (R1 dig&rarr;phys)</button>
            <button onclick="echo('physical_state')">Echo physical state (R2)</button>
            <button onclick="echo('digital_state')">Echo digital state (R2)</button>
            <button onclick="echo('environment_state')">Echo environment (R3)</button>
            <button onclick="wiring()">Show node wiring (R1)</button>
          </div>
          <pre id="evidence-out" style="display:none"></pre>
        </div>
      </section>
    </div>
  </div>

<script>
const $ = (id) => document.getElementById(id);
const text = (id, v) => { $(id).textContent = (v === null || v === undefined || v === "") ? "-" : v; };
const setMsg = (id, v) => { $(id).textContent = v; };
const f2 = (v) => (v === null || v === undefined || v === "" || isNaN(v)) ? "-" : Number(v).toFixed(2);

function shortStatus(s) {
  if (s === "TREATMENT_NEEDED") return "TREAT";
  if (s === "RETURN_SUCCEEDED" || s === "RETURNED_HOME") return "HOME";
  if (s === "SENSOR_TIMEOUT") return "TIMEOUT";
  return s || "-";
}
function shortAction(a) {
  if (a === "APPLY_TARGETED_TREATMENT") return "APPLY";
  if (a === "NO_TREATMENT") return "NONE";
  if (a === "MISSION_COMPLETE") return "DONE";
  return a || "-";
}
function statusClass(s) {
  if (s === "TREAT") return "treat";
  if (s === "HOME" || s === "HEALTHY" || s === "NO_TREATMENT_NEEDED") return "ok";
  if (s === "TIMEOUT" || s === "SENSOR_FAILED") return "bad";
  return "muted";
}
function camClass(c) {
  if (c === "healthy") return "ok";
  if (c === "degraded") return "warn";
  if (c === "failed") return "bad";
  return "muted";
}

let lastUpdate = 0;
function render(d) {
  lastUpdate = Date.now();
  const phys = d.physical_standin || {}, dig = d.digital_twin || {}, env = d.environment || {};
  const log = d.latest_log || {};
  const nav = phys.nav_feedback || {}, pose = phys.pose || {}, ctrl = d.last_control_command || {};
  const result = dig.latest_result || {}, history = d.inspection_history || [];

  text("uptime", "uptime " + (d.uptime_s ?? "-") + "s");

  const plants = history.filter(h => h.zone_id !== "plant_home");
  const done = plants.length;
  $("progress-bar").style.width = Math.min(100, (done / 6) * 100) + "%";
  text("progress-label", done + " / 6 plants");
  const final = phys.final_status;
  $("final").textContent = final ? shortStatus(final) : "in progress";
  $("final").className = "tag " + (final ? "ok" : "muted");

  // physical (incl. the digital camera health mirrored onto the physical state)
  text("p-mode", phys.mode); text("p-zone", phys.zone);
  text("p-pose", f2(pose.x) + " / " + f2(pose.y)); text("p-yaw", f2(pose.yaw));
  text("p-nav", nav.distance_remaining != null ? f2(nav.distance_remaining) + " m" : "-");
  text("p-recov", nav.number_of_recoveries ?? 0);
  const dch = phys.digital_camera_health;
  $("p-dch").innerHTML = dch ? `<span class="tag ${camClass(dch)}">${dch.toUpperCase()}</span>` : "-";

  // digital
  text("d-mode", dig.mode);
  text("d-zone", dig.mirrored_zone);
  const cam = dig.camera_health;
  $("d-cam").innerHTML = cam ? `<span class="tag ${camClass(cam)}">${cam.toUpperCase()}</span>` : "-";
  text("d-ctrl", ctrl.camera_health || "-");
  const rZone = log.zone_id || result.zone_id;
  const rStatus = shortStatus(log.status || result.status);
  const rAction = shortAction(log.recommendation || result.recommendation);
  $("d-latest").innerHTML = rZone ? `${rZone} &rarr; <span class="tag ${statusClass(rStatus)}">${rStatus}</span> ${rAction}` : "-";

  // environment
  text("e-mode", env.mode);
  $("e-obst").innerHTML = env.front_obstacle ? `<span class="tag bad">YES</span>` : `<span class="tag ok">clear</span>`;
  text("e-front", env.min_front_m != null ? f2(env.min_front_m) + " m" : "-");
  text("e-stale", env.scan_stale ? "yes" : "no");

  // results table
  const tbody = $("results");
  if (history.length) {
    tbody.innerHTML = history.map(h => {
      const home = h.zone_id === "plant_home";
      const st = shortStatus(h.status);
      const stress = home ? "-" : f2(h.plant_stress_index ?? h.disease_level);
      const camh = h.camera_health || "-";
      return `<tr>
        <td>${h.zone_id || "-"}</td>
        <td><span class="tag ${statusClass(st)}">${st}</span></td>
        <td>${stress}</td>
        <td>${shortAction(h.recommendation)}</td>
        <td><span class="tag ${camClass(camh)}">${camh}</span></td>
      </tr>`;
    }).join("");
  }

}

// keep the "updated Ns ago" pill ticking even between messages
setInterval(() => {
  if (!lastUpdate) return;
  const age = (Date.now() - lastUpdate) / 1000;
  $("age").textContent = "updated " + age.toFixed(1) + "s";
  $("age").className = "pill" + (age > 5 ? " stale" : "");
}, 200);

function connect() {
  const es = new EventSource("/events");
  es.onopen = () => { $("link").textContent = "live"; $("link").className = "pill live"; };
  es.onmessage = (e) => { try { render(JSON.parse(e.data)); } catch (_) {} };
  es.onerror = () => { $("link").textContent = "reconnecting..."; $("link").className = "pill stale"; };
}

async function postJSON(path, body) {
  return fetch(path, { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body) });
}

async function sendControl(health) {
  setMsg("control-msg", "sending camera_health=" + health + " ...");
  try {
    const r = await postJSON("/control", {camera_health: health});
    setMsg("control-msg", r.ok
      ? "Sent camera_health=" + health + " to the robot's camera. Watch it flip on the robot, mirror onto the digital twin, and change the next plant's confidence."
      : "Control failed (HTTP " + r.status + ").");
  } catch (err) { setMsg("control-msg", "Control request failed: " + err); }
}

async function obstacle(action) {
  setMsg("obstacle-msg", action === "drop" ? "spawning obstacle on the route ..." : "removing obstacle ...");
  try {
    const r = await postJSON("/obstacle", {action: action});
    setMsg("obstacle-msg", await r.text());
  } catch (err) { setMsg("obstacle-msg", "Obstacle request failed: " + err); }
}

async function echo(key) {
  const pre = $("evidence-out");
  pre.style.display = "block";
  pre.textContent = "reading " + key + " ...";
  try {
    pre.textContent = await (await fetch("/echo?topic=" + key)).text();
  } catch (err) { pre.textContent = "Echo failed: " + err; }
}

async function wiring() {
  const pre = $("evidence-out");
  pre.style.display = "block";
  pre.textContent = "running node info ...";
  try {
    pre.textContent = await (await fetch("/evidence?kind=wiring")).text();
  } catch (err) { pre.textContent = "Wiring failed: " + err; }
}

if (window.__CONTROL_ENABLED__) $("control-section").style.display = "";
connect();
</script>
</body>
</html>
"""


class _DashboardState:
    """Thread-safe bridge between the ROS node and the HTTP handler threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._latest: Optional[str] = None
        self._clients: List["queue.Queue[str]"] = []
        # Items are dicts: {"type": "camera"|"request", ...}. Drained on the ROS thread.
        self.action_requests: "queue.Queue[Dict]" = queue.Queue()
        self.control_enabled = True

    def publish(self, payload_json: str):
        with self._lock:
            self._latest = payload_json
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(payload_json)
            except queue.Full:
                pass

    def register(self) -> "queue.Queue[str]":
        q: "queue.Queue[str]" = queue.Queue(maxsize=32)
        with self._lock:
            self._clients.append(q)
            if self._latest is not None:
                q.put_nowait(self._latest)
        return q

    def unregister(self, q: "queue.Queue[str]"):
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)


def _make_handler(node: "OptionBDashboardWeb"):
    state = node.state

    class Handler(BaseHTTPRequestHandler):
        # Silence the default stderr request logging.
        def log_message(self, *args):
            pass

        def _text(self, code: int, body: str, ctype: str = "text/plain; charset=utf-8"):
            data = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _json_body(self) -> Dict:
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                return json.loads(raw.decode("utf-8") or "{}")
            except ValueError:
                return {}

        def _send_page(self):
            body = INDEX_HTML.replace(
                "window.__CONTROL_ENABLED__",
                "true" if state.control_enabled else "false",
            )
            self._text(200, body, "text/html; charset=utf-8")

        def _send_events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q = state.register()
            try:
                while True:
                    try:
                        payload = q.get(timeout=15.0)
                        self.wfile.write(b"data: " + payload.encode("utf-8") + b"\n\n")
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")  # keep-alive comment
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                state.unregister(q)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                self._send_page()
            elif parsed.path == "/events":
                self._send_events()
            elif parsed.path == "/health":
                self._text(200, "ok")
            elif parsed.path == "/echo":
                kind = parse_qs(parsed.query).get("topic", [""])[0]
                self._text(200, node.echo_topic(kind))
            elif parsed.path == "/evidence":
                if not state.control_enabled:
                    self._text(404, "actions disabled")
                    return
                kind = parse_qs(parsed.query).get("kind", ["wiring"])[0]
                self._text(200, node.run_evidence(kind))
            else:
                self._text(404, "not found")

        def do_POST(self):
            if not state.control_enabled:
                self._text(404, "actions disabled")
                return
            if self.path == "/control":
                health = str(self._json_body().get("camera_health", "")).lower()
                if health not in ("healthy", "degraded", "failed"):
                    self._text(400, "camera_health must be healthy|degraded|failed")
                    return
                state.action_requests.put({"type": "camera", "camera_health": health})
                self._text(200, json.dumps({"sent": health}), "application/json")
            elif self.path == "/obstacle":
                body = self._json_body()
                action = str(body.get("action", "")).lower()
                if action == "drop":
                    _, msg = node.spawn_obstacle(body.get("x"), body.get("y"))
                elif action == "remove":
                    _, msg = node.remove_obstacle()
                else:
                    self._text(400, "action must be drop|remove")
                    return
                self._text(200, msg)
            else:
                self._text(404, "not found")

    return Handler


class OptionBDashboardWeb(Node):
    """Serves the dashboard JSON topic as a live browser page, and drives the demo."""

    def __init__(self):
        super().__init__("option_b_dashboard_web")
        self.declare_parameter("dashboard_topic", "/dt/digital/dashboard")
        self.declare_parameter("digital_control_topic", "/dt/digital/control")
        self.declare_parameter("inspection_request_topic", "/dt/physical/inspection_request")
        self.declare_parameter("inspection_result_topic", "/dt/digital/inspection_result")
        self.declare_parameter("physical_state_topic", "/dt/physical/mission_state")
        self.declare_parameter("digital_state_topic", "/dt/digital/mission_state")
        self.declare_parameter("environment_state_topic", "/dt/physical/environment_state")
        self.declare_parameter("gz_world", "default")
        # Fixed gazebo-world drop point, set in advance on the open plant_d -> plant_e
        # leg (the robot drives along gazebo y=-2.80; the bottom wall is at y~=-3.86).
        # y is biased toward the wall so the box still covers the path but leaves the
        # upper side open to reroute. A fixed point never lands on the robot.
        self.declare_parameter("obstacle_x", 0.7)
        self.declare_parameter("obstacle_y", -3.0)
        # Big enough to clearly block the leg and force a reroute (the robot still
        # has room to go around on the open bottom leg). size_y is across the
        # corridor -- that is the dimension that actually gets in the robot's way.
        self.declare_parameter("obstacle_size_x", 0.8)
        self.declare_parameter("obstacle_size_y", 0.8)
        self.declare_parameter("obstacle_size_z", 0.7)
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("enable_control", True)

        self.state = _DashboardState()
        self.state.control_enabled = bool(self.get_parameter("enable_control").value)
        self.started_at = time.monotonic()

        # Live cache for the topic-echo buttons.
        self._cache_lock = threading.Lock()
        self._echo: Dict[str, str] = {}

        self.create_subscription(
            String, str(self.get_parameter("dashboard_topic").value), self.on_dashboard, 10
        )
        for key, (param, _default) in ECHO_TOPICS.items():
            topic = str(self.get_parameter(param).value)
            self.create_subscription(
                String, topic, lambda msg, k=key: self.on_echo(k, msg), 10
            )

        self.pub_control = self.create_publisher(
            String, str(self.get_parameter("digital_control_topic").value), 10
        )
        # Drain browser-triggered ROS publishes on the ROS thread so all
        # publishing happens on the executor, never from an HTTP handler thread.
        self.create_timer(0.1, self.drain_action_requests)

        host = str(self.get_parameter("web_host").value)
        port = int(self.get_parameter("web_port").value)
        self.httpd = ThreadingHTTPServer((host, port), _make_handler(self))
        self.httpd.daemon_threads = True
        self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.server_thread.start()

        shown = "127.0.0.1" if host in ("0.0.0.0", "") else host
        self.get_logger().info(
            f"Web dashboard at http://{shown}:{port}  "
            f"(actions {'enabled' if self.state.control_enabled else 'disabled'}). "
            "If running in Docker, publish this port (e.g. -p 8080:8080)."
        )

    # -- subscriptions ----------------------------------------------------- #
    def on_dashboard(self, msg: String):
        # Pass the raw JSON straight through; the browser parses it.
        self.state.publish(msg.data)

    def on_echo(self, key: str, msg: String):
        with self._cache_lock:
            self._echo[key] = msg.data

    def echo_topic(self, key: str) -> str:
        if key not in ECHO_TOPICS:
            return f"unknown topic key: {key}"
        topic = str(self.get_parameter(ECHO_TOPICS[key][0]).value)
        with self._cache_lock:
            raw = self._echo.get(key)
        if raw is None:
            return f"$ ros2 topic echo --once {topic}\n(no message captured yet)"
        try:
            pretty = json.dumps(json.loads(raw), indent=2, sort_keys=True)
        except ValueError:
            pretty = raw
        return f"$ ros2 topic echo --once {topic}\n{pretty}"

    # -- ROS-thread publishing of browser actions -------------------------- #
    def drain_action_requests(self):
        while True:
            try:
                item = self.state.action_requests.get_nowait()
            except queue.Empty:
                return
            kind = item.get("type")
            if kind == "camera":
                command = {
                    "event": "DIGITAL_CONTROL",
                    "source_entity": "web_dashboard",
                    "camera_health": item["camera_health"],
                    "reason": "operator fault injection from web dashboard",
                    "sent_at_uptime_s": round(time.monotonic() - self.started_at, 2),
                }
                self.pub_control.publish(String(data=json.dumps(command, sort_keys=True)))
                self.get_logger().info(
                    f"Web control -> {self.get_parameter('digital_control_topic').value}: "
                    f"camera_health={item['camera_health']}"
                )

    # -- subprocess-backed actions (safe to call from a handler thread) ----- #
    def _run(self, cmd: List[str], timeout: float = 20.0) -> Tuple[bool, str]:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                               env=os.environ)
            out = ((r.stdout or "") + (r.stderr or "")).strip()
            return r.returncode == 0, out
        except FileNotFoundError:
            return False, (f"'{cmd[0]}' not found. Run this node inside the container "
                           "with ROS/Gazebo sourced.")
        except subprocess.TimeoutExpired:
            return False, f"'{' '.join(cmd[:2])}' timed out after {timeout:.0f}s."

    def _gz_remove(self) -> Tuple[bool, str]:
        world = str(self.get_parameter("gz_world").value)
        return self._run([
            "gz", "service", "-s", f"/world/{world}/remove",
            "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", 'name: "demo_obstacle", type: MODEL',
        ])

    def _obstacle_target(self, x, y) -> Tuple[bool, float, float, str]:
        """Where to spawn, in gazebo-world coords (a fixed point on the route)."""
        if x is not None and y is not None:
            try:
                return True, float(x), float(y), "explicit"
            except (TypeError, ValueError):
                return False, 0.0, 0.0, "x and y must be numbers."
        return (True, float(self.get_parameter("obstacle_x").value),
                float(self.get_parameter("obstacle_y").value), "fixed point on the route")

    def spawn_obstacle(self, x=None, y=None) -> Tuple[bool, str]:
        ok, xv, yv, how = self._obstacle_target(x, y)
        if not ok:
            return False, how
        sx = float(self.get_parameter("obstacle_size_x").value)
        sy = float(self.get_parameter("obstacle_size_y").value)
        sz = float(self.get_parameter("obstacle_size_z").value)
        zv = sz / 2.0  # rest the box on the ground
        try:
            with open(OBSTACLE_SDF_PATH, "w") as f:
                f.write(build_obstacle_sdf(sx, sy, sz))
        except OSError as exc:
            return False, f"Could not write {OBSTACLE_SDF_PATH}: {exc}"
        # Best-effort remove any previous demo_obstacle so re-dropping never collides.
        self._gz_remove()
        world = str(self.get_parameter("gz_world").value)
        req = (f'sdf_filename: "{OBSTACLE_SDF_PATH}", '
               f'pose: {{position: {{x: {xv:.3f}, y: {yv:.3f}, z: {zv:.3f}}}}}')
        ok, out = self._run([
            "gz", "service", "-s", f"/world/{world}/create",
            "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", req,
        ])
        head = (f"Spawned {sx:.1f}x{sy:.1f}x{sz:.1f} m obstacle ({how}) at gazebo "
                f"({xv:.2f}, {yv:.2f}). Watch Environment -> OBSTACLE_AHEAD and the robot reroute."
                if ok else "Spawn call failed.")
        return ok, f"{head}\n{out}".strip()

    def remove_obstacle(self) -> Tuple[bool, str]:
        ok, out = self._gz_remove()
        head = "Removed demo_obstacle." if ok else "Remove call failed (nothing spawned?)."
        return ok, f"{head}\n{out}".strip()

    def run_evidence(self, kind: str) -> str:
        if kind == "wiring":
            parts = []
            for node_name in ("/inspection_twin_node", "/plant_nav2_mission_node"):
                _, out = self._run(["ros2", "node", "info", node_name], timeout=15.0)
                parts.append(f"$ ros2 node info {node_name}\n{out}")
            return "\n\n".join(parts)
        if kind == "topics":
            _, nodes = self._run(["ros2", "node", "list"], timeout=15.0)
            _, topics = self._run(["ros2", "topic", "list"], timeout=15.0)
            dt = "\n".join(t for t in topics.splitlines() if "/dt" in t)
            return (f"$ ros2 node list\n{nodes}\n\n"
                    f"$ ros2 topic list | grep /dt\n{dt}")
        return f"unknown evidence kind: {kind}"

    def destroy_node(self):
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OptionBDashboardWeb()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
