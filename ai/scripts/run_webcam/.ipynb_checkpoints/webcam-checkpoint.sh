#!/usr/bin/env bash
# scripts/run_webcam/webcam.sh

set -euo pipefail

# 첫 번째 인자로 config 파일 경로를 받습니다. 없으면 default.yaml
CFG=${1:-configs/default.yaml}

# 프로젝트 루트로 이동
PROJECT_ROOT=$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$PROJECT_ROOT"

# main.py 에는 -c 와 -t 옵션만 넘깁니다.
python main.py -c "$CFG" -t webcam
