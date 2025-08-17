#!/usr/bin/env bash
# =============================================================
# Gaze Detection | 하이퍼파라미터 탐색 (Optuna)
# -------------------------------------------------------------
# 사용법:
#   ./run_hpo.sh [CONFIG_PATH]
#   - 기본 config 경로: configs/default.yaml
# =============================================================

set -euo pipefail

CFG="${1:-configs/default.yaml}"
PROJECT_ROOT="$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "✅ [Gaze Detection HPO] 시작 (설정: $CFG)"
echo "프로젝트 루트: $PROJECT_ROOT"

python main.py -c "$CFG" -d gaze -t hpo

echo "🎉 하이퍼파라미터 탐색(HPO) 완료!"
