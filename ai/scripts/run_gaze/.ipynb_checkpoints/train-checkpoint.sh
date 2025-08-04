#!/usr/bin/env bash
# =============================================================
# Gaze Detection | 파인튜닝(Fine-tune) 실행 스크립트
# -------------------------------------------------------------
# 사용법:
#   ./run_finetune.sh [CONFIG_PATH]
#   - 기본 config 경로: configs/default.yaml
# =============================================================

set -euo pipefail

CFG="${1:-configs/default.yaml}"
PROJECT_ROOT="$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "✅ [Gaze Detection Fine-tune] 시작 (설정: $CFG)"
echo "프로젝트 루트: $PROJECT_ROOT"

python main.py -c "$CFG" -d gaze -t train

echo "🎉 파인튜닝(Fine-tune) 단계 완료!"
