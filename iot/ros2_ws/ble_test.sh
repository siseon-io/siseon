#!/bin/bash

# 현재 디렉토리 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🔧 Building workspace..."
# colcon build

echo "🌍 Setting up environment..."
source install/setup.bash
source .env

echo "🚀 Starting all nodes..."

# 백그라운드에서 실행
ros2 run control_bridge_node control_bridge_node &
ros2 run manual_bt_node manual_bt_node &
ros2 run pairing_bridge_node pairing_bridge_node &
ros2 run preset_bridge_node preset_bridge_node &

echo "✅ All nodes started!"
echo "🛑 Press Ctrl+C to stop all nodes."

# 사용자가 Ctrl+C를 누를 때까지 대기
trap 'echo "🛑 Stopping all nodes..."; kill $(jobs -p); exit' INT
wait