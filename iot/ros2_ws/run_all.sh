#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DEBUG_MODE=${DEBUG:-false}

echo "🔧 Building workspace..."
colcon build --packages-ignore lidar_node

echo "🌍 Setting up environment..."
# 로컬 환경(토픽/포트 등)
source .env 2>/dev/null || true

# ROS setup (필요 시 경로 수정)
ROS_DISTRO_SETUP=${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}
WS_SETUP="$SCRIPT_DIR/install/setup.bash"

[[ -f "$ROS_DISTRO_SETUP" ]] || { echo "❌ $ROS_DISTRO_SETUP not found"; exit 1; }
[[ -f "$WS_SETUP" ]] || { echo "❌ $WS_SETUP not found (build first)"; exit 1; }

echo "🚀 Starting nodes (RT/FIFO: 90/85/80)..."
ROS_ARGS="--ros-args -p debug:=$DEBUG_MODE"

run_rt () {
  local prio="$1"; shift
  local pkg="$1"; shift
  local exe="$1"; shift
  chrt -f "$prio" bash -lc "source /opt/ros/jazzy/setup.bash; source install/setup.bash; exec ros2 run '$pkg' '$exe' $ROS_ARGS" &
}

run_norm () {
  local pkg="$1"; shift
  local exe="$1"; shift
  bash -lc "source '$ROS_DISTRO_SETUP'; source '$WS_SETUP'; exec ros2 run '$pkg' '$exe' $ROS_ARGS" &
}

# RT가 유리한 노드
run_rt 90 arm_control_node arm_control_node_exec
run_rt 85 fusion_node     fusion_node
run_rt 80 eye_pose_node   eye_pose_node
run_rt 75 manual_bt_node      manual_bt_node

# 나머지 일반
run_norm control_bridge_node control_bridge_node
run_norm pairing_bridge_node pairing_bridge_node
run_norm preset_bridge_node  preset_bridge_node

# 대기
sleep 3

echo "⚙️  Setting control mode to 'off'..."
bash -lc "source '$ROS_DISTRO_SETUP'; source '$WS_SETUP'; ros2 topic pub /control_mode std_msgs/msg/String \"{data: 'off'}\" --once"

echo "✅ All nodes started (RT). Ctrl+C to stop."
trap 'echo "🛑 stopping..."; kill $(jobs -p) 2>/dev/null || true; exit' INT
wait
