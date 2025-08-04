#!/usr/bin/env bash
# =============================================================
# Upper-body YOLOv11n-Pose | TRAIN via Unified CLI
# -------------------------------------------------------------
# 사용법:
#   ./run_pose_train.sh [CONFIG_PATH]
#   - 기본 config 경로: configs/default.yaml
# =============================================================

set -euo pipefail

CFG="${1:-configs/default.yaml}"
PROJECT_ROOT="$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "✅ [Upper-body YOLOv11n-Pose TRAIN] 시작 (설정: $CFG)"
echo "프로젝트 루트: $PROJECT_ROOT"

python main.py -c "$CFG" -d pose -t train

echo "🎉 Upper-body YOLOv11n-Pose TRAIN 완료!"
