# Local changes (Windows/WSL2 + Nav2 fix)

Applied while verifying the demo on Windows 11 + WSL2 + Docker Desktop.

## 1. Nav2 fix (Dockerfile) — REQUIRED on a fresh build, all platforms
The base image's `diagnostic_updater` was older than the Nav2 packages, so
`nav2_lifecycle_manager` crashed at load:
`undefined symbol: ...diagnostic_updater::Updater...` → exit 127, and the robot never
navigated. Fix: the Dockerfile now upgrades the diagnostic stack during build
(`ros-jazzy-diagnostic-updater`, `-diagnostic-msgs`, `-diagnostic-aggregator`).
This crashed identically on every platform, so the fix matters for Linux/macOS too.

## 2. Native GUI via WSLg (docker-compose.override.yml) — Windows/WSL2 only
New `docker-compose.override.yml` binds the WSLg X11 socket and drops the internal VNC
server, so Gazebo opens as a normal window on Windows with no VNC viewer. Launch
commands are prefixed with `export DISPLAY=:0`. On macOS/Linux: delete this override
and use VNC (see README → "Running On macOS / Linux").

## 3. README updates
GUI section (WSLg vs VNC), a "Running On macOS / Linux" section, and troubleshooting
(close the demo with Ctrl-C — not the window's X button, which orphans the Gazebo
server; `docker restart turtlebot3_container` to reset stray processes).

Nothing in the ROS packages (`my_tb3_world`, `tb3_pesticide_dt`) was changed — only the
Dockerfile, a new compose override, the README, and this note.
