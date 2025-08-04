# 시선 추적 & 포즈 추정 모델

본 저장소에는 NVIDIA Jetson Nano에 최적화된 두 가지 실시간 컴퓨터 비전 서비스가 포함되어 있습니다:

* **시선 추적 (Gaze Tracking):** 눈 영역과 홍채 중심을 탐지하여 사용자의 시선 방향을 추정
* **포즈 추정 (Pose Estimation):** 관절 키포인트를 추정하여 동작 인식 및 행동 분석 지원

---

## 목차

1. [주요 기능](#주요-기능)
2. [설치 방법](#설치-방법)
3. [사용 방법](#사용-방법)

   * [CLI 워크플로우](#cli-워크플로우)
   * [웹캠 실시간 모드](#웹캠-실시간-모드)
4. [모델 상세 정보](#모델-상세-정보)

   * [시선 추적 (YOLOv11‑n)](#시선-추적-yolov11‑n)
   * [포즈 추정 (YOLOv11‑pose‑n)](#포즈-추정-yolov11‑pose‑n)
5. [데이터 소스 및 전처리](#데이터-소스-및-전처리)
6. [성능 지표](#성능-지표)
7. [라이선스](#라이선스)

---

## 주요 기능

* Jetson Nano에서 **실시간** 추론 (시선 추적 약 65ms/프레임)
* **시선 추적**과 **포즈 추정**을 단일 파이프라인으로 제공
* 사전학습 → HPO → Fine‑tune → Eval → Viz → Inference → Webcam 까지 통합 CLI
* 경량화된 YOLOv11 계열 모델로 높은 정확도와 낮은 레이턴시

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
3. (선택) 가상 환경 설정

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

---

## 사용 방법

### CLI 워크플로우

모든 단계는 `main.py` 하나로 수행합니다:

```bash
python3 main.py --config configs/default.yaml --task <단계>
```

| 단계          | 설명                                                 |
| ----------- | -------------------------------------------------- |
| `pretrain`  | Roboflow에서 시선 데이터셋 다운로드 후 사전학습                     |
| `hpo`       | Optuna 기반 하이퍼파라미터 최적화                              |
| `train`     | 사전학습+HPO 가중치 적용 후 파인튜닝                             |
| `eval`      | 학습된 모델로 `val`/`test` 평가                            |
| `viz`       | 테스트셋 샘플 5장에 바운딩박스 그려서 `runs/.../gaze/viz`에 저장      |
| `inference` | 테스트셋 폴더 단위 추론 결과 이미지(`runs/.../gaze/inference`) 생성 |
| `webcam`    | 웹캠 실시간 스트림에서 **눈 바운딩박스 + 포즈 키포인트** 함께 시각화          |
| `all`       | 위 모든 단계를 순차 실행                                     |

예시:

```bash
# 데이터 다운로드 → 학습 → 평가 → 샘플 시각화 → 파일 단위 추론 → 웹캠 모드
python3 main.py -c configs/default.yaml -t all
```

### 웹캠 실시간 모드

Qt/X11 디스플레이가 있는 환경에서만 `cv2.imshow()` 창이 뜹니다.

* **헤드리스 서버**에서는 `xvfb-run` 또는 `%matplotlib inline` 대체 출력 방식을 사용해 주세요.

```bash
# 로컬 서버나 X11 포워딩 환경
python3 main.py -c configs/default.yaml -t webcam
```

* `configs/default.yaml` 내 `inference` 섹션 예시:

  ```yaml
  inference:
    weights:       runs/train/eye_dataset/weights/best.pt
    conf_thresh:   0.25
    webcam_index:  0
    cam_width:     640
    cam_height:    480
    # pose_weights: "yolo11n-pose.pt"  # default로 하드코딩
  ```

---

## 모델 상세 정보

### 시선 추적 (YOLOv11‑n)

* **모델 유형:** 다중 객체 검출
* **알고리즘:** YOLOv11‑n
* **입력:** 640×480 RGB 이미지
* **출력:**

  * `left_eye_bbox`, `right_eye_bbox` — \[x, y, w, h]
  * `left_iris_center`, `right_iris_center` — \[x, y]
  * (`gaze_vector`는 추후 추가 예정)

### 포즈 추정 (YOLOv11‑pose‑n)

* **모델 유형:** 키포인트 검출
* **알고리즘:** YOLOv11‑pose‑n
* **입력:** 640×480 RGB 이미지
* **출력:**

  * `keypoints` — `[[x, y, conf], …]` (17개)

---

## 데이터 소스 및 전처리

* **시선 추적 데이터:**

  * GazeCapture (1,450명·2.5M+), MPIIGaze (15명·213K), CASIA‑IrisV4 (1,800명·54K)
* **포즈 추정 데이터:**

  * COCO Keypoint (60K), MPII Human Pose (25K)

**전처리 파이프라인**

1. 리사이즈 → 640×480
2. CLAHE (시선)
3. 랜덤 뒤집기·색상 변형
4. 랜덤 크롭·회전 (포즈)
5. 이상치·누락 샘플 제거

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
