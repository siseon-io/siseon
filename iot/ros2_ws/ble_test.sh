#!/bin/bash

# 현재 디렉토리 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🔧 Building workspace..."
colcon build --packages-ignore lidar_node

echo "🌍 Setting up environment..."
source install/setup.bash
source .env

echo "🚀 Starting all nodes..."

# 백그라운드에서 실행
ros2 run eye_pose_node eye_pose_node &
ros2 run manual_bt_node manual_bt_node &
ros2 run pairing_bridge_node pairing_bridge_node &
ros2 run arm_control_node arm_control_node_exec &
ros2 run fusion_node fusion_node &
# ros2 run control_bridge_node control_bridge_node &

# 노드들이 완전히 실행될 때까지 잠시 대기 (3초)
sleep 3

# 실험을 위해 제어 모드를 'manual'로 설정
echo "⚙️  Setting control mode to 'auto'..."
ros2 topic pub /control_mode std_msgs/msg/String "{data: 'auto'}" --once

echo "✅ All nodes started!"
echo "🛑 Press Ctrl+C to stop all nodes."

# 사용자가 Ctrl+C를 누를 때까지 대기
trap 'echo "🛑 Stopping all nodes..."; kill $(jobs -p); exit' INT
wait