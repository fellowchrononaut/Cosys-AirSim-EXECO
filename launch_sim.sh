#!/bin/bash
# ============================================================
#  AirSim + PX4 + QGroundControl + ROS2 — Auto Launcher
#  Uses tmux: 2 rows x 3 columns (6 panes), all in one window.
#
#  HOST   : UnrealEngine, QGroundControl
#  DOCKER : ROS2 + PX4 (container: ros2_sadvio)
#
#  Run from any terminal (tmux must be installed).
#  To install tmux:  sudo apt install tmux
# ============================================================

# ── CONFIG ──────────────────────────────────────────────────
DOCKER_CONTAINER="execo_ros2"
ROS_DISTRO="humble"
SESSION="airsim"

UNREAL_BINARY="/home/ecosense360/UnrealEngine-5.5.4-release/Engine/Binaries/Linux/UnrealEditor"
UNREAL_PROJECT="/home/ecosense360/Execo/Cosys-Airground/Cosys-AirSim-EXECO/Unreal/Environments/Blocks/Blocks.uproject"
QGC_BINARY="/home/ecosense360/Execo/QGroundControl/QGroundControl-v5.0-x86_64.AppImage"
ROS2_WS_DOCKER="/root/Cosys-AirSim-EXECO/ros2"
PX4_DIR_DOCKER="/root/PX4/PX4-Autopilot"

DELAY_UNREAL=20    # wait before ROS2 starts  (Unreal needs to be up)
DELAY_ROS=12       # wait before PX4 starts   (ROS2 needs to be up)
DELAY_PX4=8        # wait before QGC starts   (PX4 needs to be up)
# ── END CONFIG ──────────────────────────────────────────────

# ── Pre-flight checks ────────────────────────────────────────
echo "============================================"
echo "  AirSim Simulation Launcher (tmux)"
echo "============================================"

if ! command -v tmux &>/dev/null; then
    echo "  ERROR: tmux is not installed."
    echo "  Install it with:  sudo apt install tmux"
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_CONTAINER}$"; then
    echo ""
    echo "  ERROR: Docker container '${DOCKER_CONTAINER}' is not running!"
    echo "  Start it with:  docker start ${DOCKER_CONTAINER}"
    echo ""
    exit 1
fi

echo "  [OK] tmux found."
echo "  [OK] Docker container '${DOCKER_CONTAINER}' is running."
echo ""

# Kill any existing session with the same name
tmux kill-session -t "$SESSION" 2>/dev/null

# ── Cumulative delays ────────────────────────────────────────
D_ROS=$DELAY_UNREAL
D_PX4=$((DELAY_UNREAL + DELAY_ROS))
D_QGC=$((DELAY_UNREAL + DELAY_ROS + DELAY_PX4))
D_RQT=$((DELAY_UNREAL + DELAY_ROS + 3))

# ── Build the tmux layout ────────────────────────────────────
#
#  Layout (2 rows x 3 cols):
#  ┌─────────────────┬─────────────────┬─────────────────┐
#  │  [1] Unreal     │  [2] ROS2 node  │  [3] Monitor    │
#  ├─────────────────┼─────────────────┼─────────────────┤
#  │  [4] rqt        │  [5] PX4 SITL   │  [6] QGC        │
#  └─────────────────┴─────────────────┴─────────────────┘

# Create session with pane 1
tmux new-session -d -s "$SESSION" -x "220" -y "50"

# ── Row 1 ────────────────────────────────────────────────────
# Split vertically to create middle column (pane 2)
tmux split-window -h -t "$SESSION:0.0"
# Split vertically to create right column (pane 3)
tmux split-window -h -t "$SESSION:0.1"

# ── Row 2 ────────────────────────────────────────────────────
# Split each top pane horizontally to create the bottom row
tmux split-window -v -t "$SESSION:0.0"   # pane 4 (below pane 1)
tmux split-window -v -t "$SESSION:0.2"   # pane 5 (below pane 2) — note pane indices shift
tmux split-window -v -t "$SESSION:0.4"   # pane 6 (below pane 3)

# Even out the column widths
tmux select-layout -t "$SESSION" tiled

# ── Send commands to each pane ───────────────────────────────

# Pane 0 — Unreal Engine (host) — starts immediately
tmux send-keys -t "$SESSION:0.0" "
echo '━━━ [1/6] UnrealEngine ━━━'
'${UNREAL_BINARY}' '${UNREAL_PROJECT}'
" Enter

# Pane 2 — ROS2 multiagent node (docker)
tmux send-keys -t "$SESSION:0.2" "
echo '━━━ [2/6] ROS2 AirSim Multiagent Node ━━━'
echo 'Waiting ${D_ROS}s for Unreal to start...'
sleep ${D_ROS}
docker exec -it ${DOCKER_CONTAINER} bash -c '
  source /opt/ros/${ROS_DISTRO}/setup.bash &&
  source ${ROS2_WS_DOCKER}/install/setup.bash &&
  cd ${ROS2_WS_DOCKER} &&
  ros2 launch airsim_ros_pkgs airsim_node_multiagent.launch.py
'
" Enter

# Pane 4 — Monitor / free pane (host)
tmux send-keys -t "$SESSION:0.4" "
echo '━━━ [3/6] Free pane ━━━'
echo 'Use this for ad-hoc commands or log tailing.'
" Enter

# Pane 1 — rqt (docker)
tmux send-keys -t "$SESSION:0.1" "
echo '━━━ [4/6] rqt ━━━'
echo 'Waiting ${D_RQT}s for ROS2 to be ready...'
sleep ${D_RQT}
docker exec -it ${DOCKER_CONTAINER} bash -c '
  source /opt/ros/${ROS_DISTRO}/setup.bash &&
  source ${ROS2_WS_DOCKER}/install/setup.bash &&
  rqt
'
" Enter

# Pane 3 — PX4 SITL (docker)
tmux send-keys -t "$SESSION:0.3" "
echo '━━━ [5/6] PX4 SITL ━━━'
echo 'Waiting ${D_PX4}s for ROS2 to be ready...'
sleep ${D_PX4}
docker exec -it ${DOCKER_CONTAINER} bash -c '
  cd ${PX4_DIR_DOCKER} &&
  make px4_sitl_default none_iris
'
" Enter

# Pane 5 — QGroundControl (host)
tmux send-keys -t "$SESSION:0.5" "
echo '━━━ [6/6] QGroundControl ━━━'
echo 'Waiting ${D_QGC}s for PX4 to be ready...'
sleep ${D_QGC}
chmod +x '${QGC_BINARY}' && '${QGC_BINARY}'
" Enter

# ── Attach to the session ────────────────────────────────────
echo "  Attaching to tmux session '${SESSION}'..."
echo "  Tip: Press Ctrl+B then D to detach without killing anything."
echo ""
tmux attach-session -t "$SESSION"
