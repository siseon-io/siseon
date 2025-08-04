#!/usr/bin/env bash
# =============================================================
# Gaze Detection 전체 파이프라인 순차 실행 스크립트
# (pretrain → hpo → train → eval → viz → inference → webcam)
# -------------------------------------------------------------
# 사용법:
#   ./run_pipeline.sh [CONFIG_PATH]
#   - 기본 config 경로: configs/default.yaml
# =============================================================

set -euo pipefail

# 환경 설정
CFG="${1:-configs/default.yaml}"
PROJECT_ROOT="$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "✅ [Gaze Detection Pipeline] 시작 (설정: $CFG)"
echo "프로젝트 루트: $PROJECT_ROOT"

# -------------------------------
# 1) Pre-train
# -------------------------------
echo "[1/7] Pre-train 시작..."
python main.py -c "$CFG" -d gaze -t pretrain

# -------------------------------
# 2) Hyperparameter Optimization
# -------------------------------
echo "[2/7] HPO 시작..."
python main.py -c "$CFG" -d gaze -t hpo

# -------------------------------
# 3) Fine-tune (학습)
# -------------------------------
echo "[3/7] Fine-tune 시작..."
python main.py -c "$CFG" -d gaze -t train

# -------------------------------
# 4) Evaluation (평가)
# -------------------------------
echo "[4/7] Evaluation 시작..."
python main.py -c "$CFG" -d gaze -t eval

# -------------------------------
# 5) Visualization (시각화)
# -------------------------------
echo "[5/7] Visualization 시작..."
python main.py -c "$CFG" -d gaze -t viz

# -------------------------------
# 6) Folder Inference (폴더 추론)
# -------------------------------
echo "[6/7] Folder Inference 시작..."
python main.py -c "$CFG" -d gaze -t inference

# -------------------------------
# 7) Webcam Demo (웹캠 데모)
# -------------------------------
echo "[7/7] Webcam Demo 시작... (실행 중 'q' 키를 누르면 종료)"
python main.py -c "$CFG" -d gaze -t webcam

echo "🎉 전체 파이프라인 완료!"
