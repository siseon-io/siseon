#!/usr/bin/env bash
# =============================================================
# Upper-body YOLOv11n-Pose | Inference via Unified CLI
# -------------------------------------------------------------
# 사용법:
#   ./run_pose_inference.sh [CONFIG_PATH]
#   - 기본 config 경로: configs/default.yaml
# =============================================================

set -euo pipefail

CFG="${1:-configs/default.yaml}"
PROJECT_ROOT="$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "✅ [Upper-body YOLOv11n-Pose Inference] 시작 (설정: $CFG)"
echo "프로젝트 루트: $PROJECT_ROOT"

python main.py -c "$CFG" -d pose -t inference

echo "🎉 Upper-body YOLOv11n-Pose Inference 완료!"
