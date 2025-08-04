#!/usr/bin/env bash
# -------------------------------------------------------------
# Unified Webcam Demo | Gaze + Pose 실시간 데모(Webcam)
# -------------------------------------------------------------
set -euo pipefail

CFG=${1:-configs/default.yaml}
PROJECT_ROOT=$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$PROJECT_ROOT"

python main.py -c "$CFG" -t webcam
