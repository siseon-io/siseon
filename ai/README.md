# SISEON AI 서비스

본 저장소에는 NVIDIA Jetson Nano에 최적화된 실시간 컴퓨터 비전 서비스와 Chatbot 백엔드 서비스를 포함하고 있습니다.

---

## 목차

1. [주요 기능](#주요-기능)
2. [설치 방법](#설치-방법)
3. [컴퓨터 비전 사용 방법](#컴퓨터-비전-사용-방법)

   * [CLI 워크플로우](#cli-워크플로우)
   * [웹캠 실시간 모드](#웹캠-실시간-모드)
4. [모델 상세 정보](#모델-상세-정보)

   * [시선 추적 (YOLOv11‑n)](#시선-추적-yolov11‑n)
   * [포즈 추정 (YOLOv11‑pose‑n)](#포즈-추정-yolov11‑pose‑n)
5. [Chatbot 서비스](#chatbot-서비스)

   * [프로젝트 구조](#프로젝트-구조)
   * [환경 설정](#환경-설정)
   * [설치 및 실행](#설치-및-실행)
   * [API 연동 가이드](#api-연동-가이드)
6. [데이터 소스 및 전처리](#데이터-소스-및-전처리)
7. [성능 지표](#성능-지표)
8. [라이선스](#라이선스)

---

## 주요 기능

* **시선 추적 (Gaze Tracking):** 눈 영역과 홍채 중심을 탐지하여 시선 방향 추정 (약 65ms/frame)
* **포즈 추정 (Pose Estimation):** 관절 키포인트 추정으로 동작 인식 및 행동 분석
* **Chatbot 서비스:** SISEON 매뉴얼을 대상으로 RAG + LLM을 활용한 질문-응답 API (FastAPI)

---

## 설치 방법

1. 저장소 클론

   ```bash
   git clone https://lab.ssafy.com/s13-webmobile3-sub1/S13P11B101.git
   cd S13P11B101/AI
   ```
2. 의존성 설치 (Jetson Nano / Ubuntu 18.04+)

   ```bash
   sudo apt update
   sudo apt install python3-pip libopencv-dev
   pip3 install -r requirements.txt
   ```
3. (선택) 가상 환경

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

---

## 컴퓨터 비전 사용 방법

### CLI 워크플로우

```bash
python3 main.py --config configs/default.yaml --task <단계>
```

| 단계          | 설명                       |
| ----------- | ------------------------ |
| `pretrain`  | 데이터 다운로드 및 사전 학습         |
| `hpo`       | Optuna 기반 하이퍼파라미터 최적화    |
| `train`     | 사전학습 + HPO 가중치 적용 후 파인튜닝 |
| `eval`      | `val`/`test` 평가          |
| `viz`       | 샘플 시각화                   |
| `inference` | 파일 단위 추론 결과 생성           |
| `webcam`    | 웹캠 실시간 모드                |
| `all`       | 전체 단계 순차 실행              |

### 웹캠 실시간 모드

```bash
python3 main.py -c configs/default.yaml -t webcam
```

`configs/default.yaml` 예시:

```yaml
inference:
  weights: runs/train/eye_dataset/weights/best.pt
  conf_thresh: 0.25
  webcam_index: 0
  cam_width: 640
  cam_height: 480
```

---

## 모델 상세 정보

### 시선 추적 (YOLOv11‑n)

* 입력: 640×480 RGB 이미지
* 출력:

  * `left_eye_bbox`, `right_eye_bbox` — \[x, y, w, h]
  * `left_iris_center`, `right_iris_center` — \[x, y]

### 포즈 추정 (YOLOv11‑pose‑n)

* 입력: 640×480 RGB 이미지
* 출력:

  * `keypoints` — \[\[x, y, conf], …] (17개)

---

## Chatbot 서비스

### 프로젝트 구조

```
siseon/ai/chatbot/
├── manuals/                       # RAG용 PDF 매뉴얼
│   └── siseon_manual.pdf
├── config.yaml                    # 설정 (API 키, 모델명 등)
├── .env                           # (선택) 환경변수
├── requirements.txt               # Python 의존성 목록
├── settings.py                    # 설정 로딩 및 검증
├── llm.py                         # GMSChat LLM 래퍼
├── indexer.py                     # RAG 인덱싱
├── service.py                     # 질문-응답 서비스 로직
├── app.py                         # FastAPI 엔드포인트 (/chat)
└── test_app.py                    # 테스트 스크립트
```

### 환경 설정

#### config.yaml

```yaml
openai:
  api_key: "<YOUR_OPENAI_API_KEY>"
  api_base: "https://gms.ssafy.io/gmsapi/api.openai.com/v1/responses"
model:
  name: "gpt-3.5-turbo"
  temperature: 0.0
manual_path: "manuals/siseon_manual.pdf"
```

* `.env`로 오버라이드 가능

### 설치 및 실행

```bash
cd siseon/ai/chatbot
pip install -r requirements.txt
uvicorn app:app --reload --host 0.0.0.0 --port 8000
```

### 테스트

````bash
python test_app.py
```  또는 curl:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"question":"SISEON 매뉴얼의 모델 이름은?"}'
````

### API 연동 가이드

* **기본 URL:** `http://localhost:8000`
* **POST /chat** 요청: `{ "question": string }` → 응답: `{ "summary": string, "details": dict }`
* **Swagger UI:** `http://localhost:8000/docs`
* **OpenAPI JSON:** `http://localhost:8000/openapi.json`

---

## 데이터 소스 및 전처리

* 시선 데이터: GazeCapture, MPIIGaze, CASIA‑IrisV4
* 포즈 데이터: COCO Keypoint, MPII Human Pose

---

## 성능 지표

| 지표                     | 값            |
| ---------------------- | ------------ |
| Eye Detection AP\@0.50 | --           |
| Pose Keypoint AP       | --           |
| 실시간 지연 (Jetson Nano)   | \~65ms/frame |

---

## 라이선스

본 프로젝트는 **OO 라이선스** 하에 배포됩니다.
