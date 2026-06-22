# Reproducible image for the CBL Option B Digital Twin demo.
#
# The image contains ROS 2 Jazzy, Gazebo, Nav2, TurtleBot3 and a VNC desktop.
# The project source is mounted at runtime into /ws/src/cbl_option_b, so the
# same image works from Windows/WSL, Linux and macOS/Colima without hard-coded
# host paths.
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# ---- ROS / Gazebo / Nav2 dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ca-certificates build-essential python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-msgs \
    ros-jazzy-xacro \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# ---- TurtleBot3 sources built into the image's own workspace ----
WORKDIR /opt/turtlebot3_ws
RUN mkdir -p src && cd src && \
    git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --rosdistro jazzy --from-paths src --ignore-src -r -y || true && \
    colcon build --event-handlers console_direct+ \
      --packages-select \
        turtlebot3_msgs \
        turtlebot3_description \
        turtlebot3_gazebo \
        turtlebot3_teleop \
        turtlebot3_navigation2

# ---- Headless GUI: VNC server + a minimal desktop + software OpenGL ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    tigervnc-standalone-server tigervnc-common tigervnc-tools \
    fluxbox xterm xauth x11-xserver-utils x11-utils \
    mesa-utils procps less nano \
    && rm -rf /var/lib/apt/lists/*

# ---- Fix Nav2 / diagnostic_updater ABI mismatch ----
# The osrf/ros:jazzy-desktop-full base ships an older diagnostic_updater than the
# nav2 packages installed above; nav2_lifecycle_manager then dies at load time with
# "undefined symbol: ...diagnostic_updater::Updater...". Upgrade the diagnostic
# stack so the symbol resolves and Nav2 can start.
RUN apt-get update && apt-get install -y --only-upgrade \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-diagnostic-aggregator \
    && rm -rf /var/lib/apt/lists/*

# Pre-set the VNC password ("ros") and a minimal window-manager session so
# Gazebo and RViz windows are movable inside the VNC desktop.
RUN mkdir -p /root/.vnc && \
    echo "ros" | tigervncpasswd -f > /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd && \
    printf '#!/bin/sh\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\nfluxbox &\nexec xterm\n' \
      > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup

# ---- Environment ----
ENV DISPLAY=:1
ENV VNC_PORT=5901
ENV VNC_RESOLUTION=1280x800
ENV VNC_COL_DEPTH=24
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=36

COPY docker/start-vnc /usr/local/bin/start-vnc
COPY docker/ros-env /usr/local/bin/ros-env
RUN chmod +x /usr/local/bin/start-vnc /usr/local/bin/ros-env && \
    mkdir -p /ws/src

RUN echo "source /opt/ros/jazzy/setup.bash"              >> /root/.bashrc && \
    echo "source /opt/turtlebot3_ws/install/setup.bash"  >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger"                >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=36"                       >> /root/.bashrc && \
    echo "export DISPLAY=:1"                             >> /root/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1"                >> /root/.bashrc

WORKDIR /ws
EXPOSE 5901 8080

CMD ["/bin/bash"]
