#!/usr/bin/env bash
# install/share/pairing_node/local_setup.sh
# ROS2 install/setup.bash 에서 자동으로 source 됩니다

# AMENT_PREFIX_PATH 의 첫 entry를 install 경로로 보고 워크스페이스 루트 찾기
if [ -n "$AMENT_PREFIX_PATH" ]; then
  WS_INSTALL_DIR="${AMENT_PREFIX_PATH%%:*}"
  WS_ROOT="$(dirname "$WS_INSTALL_DIR")"
else
  WS_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
fi

ENV_FILE="$WS_ROOT/.env"
if [ -f "$ENV_FILE" ]; then
  set -a
  source "$ENV_FILE"
  set +a
  echo "✅ Loaded .env from $ENV_FILE"
fi
