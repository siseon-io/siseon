# SISEON: 인체공학 기반 디스플레이 제어 시스템

## 💡 프로젝트 소개

**SISEON**(Sight-based Intelligent System Elevation Optimization Normalizer)은 사용자의 눈 위치와 자세를 실시간으로 인식하여, 모니터 위치(X/Y/Z 축)를 자동으로 조정하는 **인체공학 기반 디스플레이 제어 시스템**입니다.

- Jetson Orin Nano 기반 AI 모듈
- Raspberry Pi 5 제어 시스템
- SCARA 방식 3축 로봇 암
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
[App UI] ⇄ [Spring API] ⇄ [Http Module (Pi5)] ⇄ [UART] ⇄ [Jetson]
                                           ↓
                                   [SCARA 모니터 암]
```

---

## 동작 시나리오

### 1. 프리셋 적용
1. 사용자 앱에서 프리셋 선택
2. Spring API에 요청 저장
3. Http Module(Pi5)로 polling 후 제어 명령 실행
4. Pi5가 Arm 제어 → 디스플레이 이동

### 2. 자동 제어
1. Jetson 실시간 추론 결과 전송
   - Eye Detection 결과 (눈 위치 y, z축 좌표) 전송
   - Pose Estimation 결과 (상반신 관절 단위 y, z축 좌표) 전송
2. Pi가 Jetson 결과에 따라 Arm x, y, z축 제어
3. Flutter가 App에서 누적된 데이터 바탕으로 사용자 대시보드 제공
4. (Option) 서버에 프리셋 저장 요청

### 3. 수동 제어
1. Flutter 앱에서 수동으로 지정
2. Flutter 앱에서 조이스틱 (x, y 1개, z 1개) 으로 조정
3. Http Module(Pi5)로 POST 요청 받아서 제어 명령 실행
4. (Option) 서버에 프리셋 저장 요청

---

## 설치 및 실행 방법

### 1. Jetson Orin Nano
- **OS**: Ubuntu 20.04 + JetPack 5.x
- **설치**: `requirements.txt` + 카메라 설정
- **AI 모델**: PyTorch, OpenCV, Ultralytics YOLO

### 2. Raspberry Pi 5
- **OS**: VxWorks OS (64-bit) → RTOS
- **설치**: Http Module + GPIO 제어 서비스 등록
- **펌웨어**: C/C++ + GPIO/PWM 라이브러리

### 3. EC2 서버
- **OS**: Ubuntu 22.04 + OpenJDK 17 + Spring Boot
- **설치**: Spring Boot 배포, HTTPS 설정, RDS 연결
- **백엔드**: Java (Spring Boot)

### 4. User Device
- **플랫폼**: Android
- **앱**: Flutter 기반 UI

---

## 기술 스택

- **Backend**: Java (Spring Boot)
- **Frontend**: Flutter
- **Firmware**: C/C++ (GPIO, PWM)
- **AI**: PyTorch, OpenCV, Ultralytics YOLO
- **DB**: AWS RDS (MySQL 등)

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
- **Jetson ↔ Pi5**: UART 3.3V TTL, 115200bps
- **Pi5 ↔ 모니터암**: GPIO (PWM, Relay), I2C (PCA9685)
- **서보모터**: MG996R (PWM 제어, 6V 전원)
- **리니어 액추에이터**: 12V + 릴레이 구동

### 소프트웨어 인터페이스
- **Pi5 Http Module**:
  - `POST /apply-preset`: 프리셋 적용 명령
  - `POST /move-arm`: 수동 좌표 이동
- **Jetson**:
  - `UART Packet: EYE_POS,x,y,z`
  - `UART Command: RESET / RE-CALIBRATE`
- **EC2 API**:
  - `POST /api/preset`
  - `POST /api/monitoring`
  - `GET /api/user/history`

### 통신 인터페이스
- **Jetson ↔ Pi5**: UART
- **Pi5 ↔ EC2**: HTTP REST API
- **Http Module ↔ Flutter**: CORS 허용 REST

---

## 성능 요구사항

- 실시간 위치 반영 딜레이 ≤ 300ms
- 프리셋 적용 후 모니터 도달 시간 ≤ 10초
- Jetson AI 모델 프레임 속도 ≥ 15 FPS
- EC2는 초당 프리셋 요청 50 TPS 처리 가능
- Arm 이동 속도: 0.5 cm/s
- AI 추론 시간: 33 ~ 60 ms/frame

---

## 보안 요구사항

- Pi5, Jetson은 LAN 내부에서만 접근 가능하도록 방화벽 구성
- Http Module는 JWT 기반 인증 적용 예정
- EC2 Spring 서버는 HTTPS + API Key 인증 적용
- 앱 → EC2 통신은 TLS 1.2 이상 사용
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
