# 시선 추적 & 포즈 추정 모델

본 저장소에는 NVIDIA Jetson Nano에 최적화된 두 가지 실시간 컴퓨터 비전 서비스가 포함되어 있습니다:

* **시선 추적 (Gaze Tracking):** 눈 영역과 홍채 중심을 탐지하여 사용자의 시선 방향을 추정
* **포즈 추정 (Pose Estimation):** 관절 키포인트를 추정하여 동작 인식 및 행동 분석 지원

---

## 목차

1. [주요 기능](#주요-기능)
2. [설치 방법](#설치-방법)
3. [사용 방법](#사용-방법)

   * [시선 추적 API](#시선-추적-api)
   * [포즈 추정 API](#포즈-추정-api)
4. [모델 상세 정보](#모델-상세-정보)

   * [시선 추적 (YOLOv11‑n)](#시선-추적-yolov11‑n)
   * [포즈 추정 (YOLOv11‑pose‑n)](#포즈-추정-yolov11‑pose‑n)
5. [데이터 소스 및 전처리](#데이터-소스-및-전처리)
6. [성능 지표](#성능-지표)
7. [라이선스](#라이선스)

---

## 주요 기능

* Jetson Nano에서 **실시간** 추론 (시선 추적 약 65ms/프레임)
* 시선 추적 및 포즈 추정을 위한 **듀얼 서비스** 제공
* 경량화된 모델로 높은 정확도 보장

---

## 설치 방법

1. 저장소 클론

   ```bash
   git clone https://lab.ssafy.com/s13-webmobile3-sub1/S13P11B101.git
   cd AI
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

서버 실행:

```bash
python3 app.py  # 기본 포트 8000에서 시작
```

### 시선 추적 API

* **POST** `/api/v1/predict-gaze`
* **요청** base64 인코딩된 640×480 RGB 이미지

  ```json
  { "image": "<base64-encoded frame>" }
  ```
* **응답**

  ```json
  {
    "left_eye_bbox": [120, 80, 60, 40],
    "right_eye_bbox": [260, 82, 58, 38],
    "left_iris_center": [150, 100],
    "right_iris_center": [290, 102],
    "gaze_vector": [0.12, -0.04, 0.99]
  }
  ```

### 포즈 추정 API

* **POST** `/api/v1/predict-pose`
* **요청** base64 인코딩된 640×480 RGB 이미지

  ```json
  { "image": "<base64-encoded frame>" }
  ```
* **응답**

  ```json
  {
    "people_count": 1,
    "keypoints": [
      [[100,50,0.98], [120,70,0.95], … 17개],
      [[200,60,0.96], [220,80,0.93], … 17개]
    ]
  }
  ```

---

## 모델 상세 정보

### 시선 추적 (YOLOv11‑n)

* **모델 유형:** 이진/다중 객체 검출
* **알고리즘:** YOLOv11‑n
* **입력:** 640×480 RGB 이미지
* **출력:** 눈 바운딩박스, 홍채 중심, 3D 시선 벡터

| 필드                  | 타입 | 설명                      |
| ------------------- | -- | ----------------------- |
| left\_eye\_bbox     | 배열 | \[x, y, w, h] 왼쪽 눈 바운딩  |
| right\_eye\_bbox    | 배열 | \[x, y, w, h] 오른쪽 눈 바운딩 |
| left\_iris\_center  | 배열 | \[x, y] 왼쪽 홍채 중심        |
| right\_iris\_center | 배열 | \[x, y] 오른쪽 홍채 중심       |
| gaze\_vector        | 배열 | \[dx, dy, dz] 정규화된 벡터   |

### 포즈 추정 (YOLOv11‑pose‑n)

* **모델 유형:** 다중 객체 검출
* **알고리즘:** YOLOv11‑pose‑n
* **입력:** 640×480 RGB 이미지
* **출력:** 사람 수 및 17개 키포인트

| 필드            | 타입 | 설명                       |
| ------------- | -- | ------------------------ |
| people\_count | 정수 | 검출된 사람 수                 |
| keypoints     | 배열 | \[x, y, confidence] × 17 |

---

## 데이터 소스 및 전처리

* **시선 추적 데이터:** GazeCapture(1,450명•2.5M+), MPIIGaze(15명•213K), CASIA‑IrisV4(1,800명•54K)
* **포즈 추정 데이터:** COCO Keypoint(60K), MPII Human Pose(25K)

**전처리:**

1. 해상도 640×480
2. CLAHE (시선)
3. 랜덤 뒤집기·색상 변형
4. 이상치 제거
5. 랜덤 크롭·회전 (포즈)
6. 누락 샘플 제외

---

## 성능 지표

| 지표                     | 값     |
| ---------------------- | ----- |
| Eye Detection AP\@0.50 | -  |
| 추론 지연 시간 (Jetson)      | - |

---

## 라이선스

OO 라이선스 하에 배포됩니다.