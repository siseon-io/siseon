# SISEON: 인체공학 기반 디스플레이 제어 시스템

## 💡 프로젝트 소개

**SISEON**(Sight-based Intelligent System Elevation Optimization Normalizer)은 사용자의 눈 위치와 자세를 실시간으로 인식하여, 모니터 위치(X/Y/Z 축)를 자동으로 조정하는 **인체공학 기반 디스플레이 제어 시스템**입니다.

- Jetson Orin Nano 기반 AI 모듈
- Raspberry Pi 5 제어 시스템
- 5DOF 오픈매니퓰레이터 (OpenMANIPULATOR-X, RM-X52-TNM)
- Spring Boot API 서버 및 Flutter 앱
- AWS RDS 기반 프리셋/로그 저장

---

## 주요 기능

- AI 기반 사용자 위치 인식 및 자동 조절
- 디바이스 위치 자동 조정 (XYZ축 이동)
- 프리셋 저장 및 호출
- 모바일 앱(Flutter) 제어 및 실시간 상태 모니터링
- 펌웨어 OTA 업데이트
- 자세 모니터링 및 통계 시각화

---

## 시스템 구성도

```
[Flutter App] ↔ [Spring API Server (EC2)]
    ↕ (BLE, MQTT)                ↕ (HTTP, MQTT)
[Raspberry Pi 5 (ROS2 Nodes)] ↔ [OpenMANIPULATOR-X]
    ↑ (UDP Socket)
[Jetson Nano]
```

---

## 동작 시나리오

### 1. 프리셋 적용
1. 사용자 앱에서 프리셋 선택
2. Spring API 서버에 프리셋 적용 요청 (HTTP)
3. Spring 서버가 MQTT Topic(`/preset_coordinate/{device_id}`)으로 프리셋 좌표 전송
4. Pi의 `preset_bridge_node`가 메시지를 수신하여 `fusion_node`로 전달
5. 최종 제어 명령이 `arm_control_node`를 통해 매니퓰레이터로 전송

### 2. 자동 제어
1. Jetson이 실시간 추론 결과(눈/자세 좌표)를 Pi로 전송 (UDP Socket)
2. Pi의 `eye_pose_node`가 데이터를 수신하여 ROS2 Topic(`/eye_pose`)으로 발행
3. `fusion_node`가 눈 위치 데이터를 기반으로 목표 위치 계산
4. Pi는 실시간 자세 데이터를 Spring 서버로 전송 (MQTT)
5. 최종 제어 명령이 `arm_control_node`를 통해 매니퓰레이터로 전송

### 3. 수동 제어
1. Flutter 앱에서 조이스틱으로 수동 조작
2. 앱이 Pi의 `manual_bt_node`와 직접 통신(BLE)하여 실시간 제어
3. (Option) 현재 위치를 서버에 프리셋으로 저장 요청 (HTTP)

---

## 설치 및 실행 방법

### 1. Jetson Nano
- **OS**: Ubuntu 22.04+ (JetPack 4.x)
- **설치**: `requirements.txt` + 카메라 설정
- **AI 모델**: PyTorch, OpenCV, Ultralytics YOLOv11

### 2. Raspberry Pi 5
- **OS**: Ubuntu 24.04 LTS (64-bit)
- **설치**: ROS2 Jazzy, DYNAMIXEL SDK, MQTT 클라이언트 라이브러리
- **펌웨어**: C++ (ROS2 기반 노드)

### 3. EC2 서버
- **OS**: Ubuntu 22.04 + OpenJDK 17 + Spring Boot
- **설치**: Spring Boot 배포, HTTPS 설정, RDS 연결
- **백엔드**: Java (Spring Boot)

### 4. User Device
- **플랫폼**: Android
- **앱**: Flutter 기반 UI

---

## 기술 스택

- **Backend**: Java (Spring Boot), Spring Batch, MQTT(EMQX), Jenkins, Docker
- **Frontend**: Flutter, BLE(`flutter_blue_plus`), MQTT(`mqtt_client`)
- **Firmware**: C++ (ROS2, DYNAMIXEL SDK)
- **AI**: PyTorch, OpenCV, Ultralytics YOLOv11, FastAPI
- **DB**: AWS RDS (MySQL)

---

## 디렉토리 구조

```
S13P11B101/
├── ai/         # AI 모델 및 추론 코드 (Jetson)
├── backend/    # Spring Boot API 서버
├── frontend/   # Flutter 앱
├── iot/        # Pi5 제어 모듈 및 펌웨어
└── README.md   # 프로젝트 문서
```

---

## 주요 인터페이스

### 하드웨어 인터페이스
- **Jetson ↔ Pi5**: UDP Socket
- **Pi5 ↔ 매니퓰레이터**: DYNAMIXEL Protocol (TTL Half Duplex UART)
- **액추에이터**: DYNAMIXEL XM430-W210-T

### 소프트웨어 인터페이스

#### MQTT Topics (App/Server ↔ Pi5)
- **`/control_mode/{device_id}`**: 제어 모드(auto/manual/preset) 변경
- **`/preset_coordinate/{device_id}`**: 프리셋 좌표 전송
- **`/request_pair/{device_id}`**: 디바이스 페어링 요청

#### ROS2 Topics (Pi5 내부 노드 간 통신)
- **`/eye_pose`**: Jetson에서 수신한 실시간 눈 좌표
- **`/manual_pose`**: 앱(BLE)에서 수신한 수동 제어 좌표
- **`/preset_pose`**: MQTT로 수신한 프리셋 좌표
- **`/cmd_pose`**: 최종 계산된 매니퓰레이터 목표 위치

#### REST API (App ↔ Server)
- **`POST /api/auth/login`**: 로그인 (JWT 발급)
- **`POST /api/presets`**: 프리셋 저장
- **`GET /api/stats`**: 자세 통계 조회

### 통신 인터페이스
- **Jetson ↔ Pi5**: UDP Socket
- **Pi5 ↔ EC2 Server**: MQTT (TLS), HTTP
- **Flutter ↔ Pi5**: BLE, MQTT (TLS)
- **Flutter ↔ EC2 Server**: HTTPS (REST API)

---

## 성능 요구사항

- 실시간 위치 반영 딜레이 ≤ 300ms
- 프리셋 적용 후 모니터 도달 시간 ≤ 10초
- Jetson AI 모델 프레임 속도 ≥ 15 FPS
- EC2는 초당 프리셋 요청 50 TPS 처리 가능
- Arm 이동 속도: 112 rpm
- AI 추론 시간: 33 ~ 50 ms/frame

---

## 보안 요구사항

- Pi5, Jetson은 LAN 내부에서만 접근 가능하도록 방화벽 구성
- API 서버 및 Pi 제어 모듈은 JWT 기반 인증 적용
- EC2 Spring 서버는 HTTPS + API Key 인증 적용
- 앱 ↔ 서버, 앱 ↔ Pi(MQTT) 통신은 TLS 1.2 이상 사용
- 카메라 이미지 미저장 원칙

---

## 설계 및 구현 제약사항

### 하드웨어 제약
- 구동 각도 및 길이에 따라 모터 토크 필요량 증가
- 무게 하중에 따라 부품 강성 및 정밀도 요구
- On-device AI 구현에 필요한 GPU (SoC 8Gb)

### 소프트웨어 제약
- 실시간 제어를 위한 통신 지연 최소화 필요
- 오프라인 모드 대응을 위한 로컬 저장 로직 필요
- 전체 시스템은 실시간 반응성을 보장해야 함 (<300ms)

---

## 개발 환경

- **Unit Test**: pytest (Pi5), JUnit (Spring)
- **Integration Test**: 시뮬레이터 기반 모니터 암 가상 제어
- **Real-world Test**: Jetson 카메라 + 설계한 모니터 암

---

## 구성 관리

- **GitLab** (Private Repo)
  - `/backend/`
  - `/ai/`
  - `/frontend/`
  - `/iot/`
- **Notion** → 문서 정리
- **Jira** → 개발 일정 관리

---

## 문서 및 매뉴얼

- 설치 가이드 PDF (Ubuntu 환경 설정, GPIO 연결법 포함)
- 앱 UI 매뉴얼 (페이지별 기능 설명서)
- 유지보수 가이드 (API 목록, 오류 코드, 시스템 재시작 프로세스)

---

## 참고 자료

- [NVIDIA Jetson Developer Guide](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [Spring Boot 공식 문서](https://docs.spring.io/spring-boot/docs/current/reference/html/)
- [Ultralytics YOLOv11](https://docs.ultralytics.com/)
- IEEE 830 Software Requirements Specification

---

## 호환성

- 추후 리니어 구조 모니터암 또는 XY슬라이더 구조에도 호환되도록 소프트웨어는 **제어 모듈 추상화 구조로 설계**
- 프리셋 구조는 버전 관리 가능하도록 JSON 기반 설계

---

## 문의 및 기여

이 프로젝트에 대한 문의, 버그 제보, 기여는 담당자 또는 이슈 트래커를 통해 남겨주세요.
