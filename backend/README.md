# SISEON 모니터암 제어 백엔드

**시선(Siseon)**은 사용자의 자세 데이터를 실시간으로 수집하고, 통계 분석을 통해 바른 자세를 유도하며,  
프리셋을 기반으로 모니터암을 자동 제어하는 IoT 백엔드 시스템입니다.

> 실시간 수집 → 누적 저장 → 통계 분석 → 모니터 제어 & 피드백 → 푸시 알림까지  
> 모든 과정을 Spring 기반으로 자동화합니다.

---

## 🌐 전체 서비스 흐름 요약

1. Raspberry Pi에서 사용자의 자세 데이터를 MQTT로 전송  
2. EMQX 브로커를 통해 Backend가 MQTT 메시지를 수신  
3. 수신된 PoseData를 실시간 저장  
4. 30분마다 Spring Batch가 실행되어 시간 구간별 통계 생성  
5. 사용자 요청 시 프리셋 적용 → Pi 서버를 통해 모니터암 제어  
6. 장시간 고정된 자세 유지 시 Firebase 푸시 알림 발송  

---

## 📡 시스템 아키텍처 흐름도

![alt text](image-1.png)

```
[ Raspberry Pi ]
       │
  MQTT 송신 (pose/data)
       ↓
[ EMQX Broker (TLS 8883) ]
       ↓
[ Spring Backend ]
       ├─> PoseData 저장 (main DB)
       ├─> 프리셋 API 호출 → Pi 서버
       ├─> Batch Job 실행 (30분 단위)
       │     └─> PostureStats 저장 (batch DB)
       └─> Firebase Push 알림 전송
```

---

## 🔁 주요 처리 흐름

### 1. MQTT 처리

- 브로커: EMQX
- 포트: 8883 (TLS)
- 주요 토픽  
  - `pose/data`: 실시간 자세 데이터 전송  
  - `pi/control`: 프리셋 제어 요청  
- 수신 메시지는 PoseData 엔티티로 저장됨

### 2. 실시간 데이터 저장

- `PoseData`는 main 스키마에 저장되며  
  사용자, 시간, 자세 좌표 (x, y, z)를 포함

### 3. 배치 통계 처리

- Spring Batch Job이 30분마다 실행됨
- `raw_postures`에서 데이터를 읽어 Slot index 계산
- 해당 구간의 평균값(avgx, avgy, avgz) → `posture_stats`에 저장
- 특정 조건 만족 시 Firebase 푸시 알림 전송

### 4. 프리셋 기반 제어

- 사용자 요청 시 등록된 프리셋 좌표를 기반으로
- Pi 서버에 HTTP 요청 전송 → 모터 제어 수행

---

## ⚙️ 기술 스택

| 항목         | 기술                                         |
|--------------|----------------------------------------------|
| 백엔드       | Spring Boot 2.5.5, Spring Batch              |
| DB           | MySQL (main, batch 스키마 분리)              |
| 메시징       | MQTT (EMQX Broker, TLS 8883)                 |
| 푸시 알림    | Firebase Cloud Messaging (FCM)               |
| 배포         | Jenkins, Docker, AWS EC2                     |
| 보안         | JWT 인증, OAuth2 (Google)                    |
| 엣지 서버     | Flask (Pi 모터 제어용)                      |
| 클라이언트 앱 | Flutter (Android / iOS)                      |

---

## 📁 디렉터리 구조

```
📁 siseon-backend/
├── 📄 Dockerfile
├── 📄 docker-compose.yml
├── 📄 .env
├── 📄 build.gradle
├── 📄 Jenkinsfile
├── 📄 settings.gradle
├── 📁 src
│   └── 📁 main
│       ├── 📁 java
│       │   └── 📁 siseon.backend
│       │       ├── 📄 SiseonBackendApplication.java
│       │       ├── 📁 config              # CORS, JWT, DB, Batch 설정
│       │       ├── 📁 domain
│       │       │   ├── 📁 main            # 사용자, 프리셋, 자세 데이터 등
│       │       │   └── 📁 batch           # 통계(PostureStats) 관련 도메인
│       │       ├── 📁 repository
│       │       │   ├── 📁 main            # 실시간 DB용 JPA 리포지토리
│       │       │   └── 📁 batch           # 배치 DB용 JPA 리포지토리
│       │       ├── 📁 service             # 비즈니스 로직
│       │       ├── 📁 controller          # REST API 엔드포인트
│       │       ├── 📁 batch               # Spring Batch Job 구성
│       │       └── 📁 mqtt                # MQTT 메시지 수신 처리
│       └── 📁 resources
│           ├── 📄 application.yml
│           ├── 📄 application-dev.yml
│           └── 📄 application-prod.yml
└── 📁 test                               # 단위 및 통합 테스트
```

---

## 🧩 주요 기능 요약

| 기능          | 설명                                                             |
|---------------|------------------------------------------------------------------|
| 사용자 인증    | JWT 기반 인증, OAuth2 Google 로그인 지원                         |
| 프리셋 관리    | 프리셋 등록 및 제어, Pi 서버로 제어 명령 전송                    |
| 자세 데이터    | MQTT를 통해 수신 후 PoseData에 저장                              |
| 통계 생성      | 30분 단위 Slot별 평균 통계 생성 (Spring Batch)                   |
| 푸시 알림      | 장시간 동일 자세 유지 시 Firebase로 사용자 알림 전송             |
| 대시보드       | 사용자별 일/주/월 통계 조회 API 제공                              |
| 배포 및 구성   | Jenkins CI/CD, Docker Compose 기반 배포 자동화                   |

---

## 핵심 로직별 상세 설명

### 1. 실시간 자세 데이터 수집 및 저장

**목적**  
사용자의 실시간 자세를 라즈베리파이 디바이스에서 10Hz 단위로 수집하고, 이를 백엔드 서버에 안전하게 전달 및 저장합니다.

**구성 요소**  
- Raspberry Pi + 센서 모듈 (OpenPose 기반)
- MQTT 브로커(EMQX)
- Backend MQTT Subscriber (`MqttPoseListener`)
- JPA 기반 `PoseData` 엔티티

**동작 방식**
1. 라즈베리파이에서 OpenPose 등으로 분석된 3차원 자세 데이터(x, y, z)가 MQTT 메시지 형태로 전송됩니다.
2. MQTT 브로커(EMQX)는 TLS 8883 포트를 통해 인증된 메시지만 수신합니다.
3. Spring Boot 백엔드는 `pose/data` 토픽에 대한 구독자(subscriber)를 실행하고 있으며, `MqttCallback`을 통해 메시지를 수신합니다.
4. 수신된 메시지를 파싱하여 `PoseData` 객체로 변환한 뒤, main 스키마의 DB에 영속화합니다.

**기술적 고려 사항**
- TLS 기반 MQTT 보안 통신
- 메시지 유실 방지를 위한 QoS 설정 (보통 QoS 1 적용)
- 최대 10Hz 처리 부하 대응을 위한 비동기 저장 처리 또는 버퍼링 설계 고려 가능

---

### 2. 프리셋 기반 모니터암 제어

**목적**  
사용자가 원하는 자세(프리셋)를 선택하면, 해당 좌표를 Raspberry Pi에 전달하여 모터를 해당 위치로 제어합니다.

**구성 요소**  
- Preset 엔티티 (이름, XYZ 좌표, 소유자 정보 등 포함)
- `PresetController` → `PiControlService`
- Pi 엣지 서버 (Flask 기반)

**동작 방식**
1. 사용자가 Flutter 앱을 통해 프리셋을 선택하면, REST API 요청(`/api/presets/{id}/move`)이 백엔드로 전송됩니다.
2. 백엔드에서는 해당 프리셋 정보를 조회한 후 `RestTemplate` 또는 `WebClient`를 통해 Pi 서버에 HTTP POST 요청을 보냅니다.
3. Pi 서버는 요청을 수신하고, 해당 좌표로 모터 제어 명령을 수행합니다.

**기술적 고려 사항**
- 프리셋 적용 실패에 대비해 Spring Retry 또는 fallback 정책 설정
- 모터 제어 실패/성공 결과에 따라 클라이언트 응답 분기

---

### 3. Batch 기반 30분 단위 통계 처리

**목적**  
실시간으로 수집된 PoseData를 일정 시간 단위로 요약하여, 사용자에게 통계 및 피드백을 제공합니다. 저장 효율성과 통계 조회 속도 최적화를 위해 배치 처리 방식을 채택합니다.

**구성 요소**  
- `raw_postures` 테이블 (수집된 PoseData 원본)
- `posture_stats` 테이블 (통계 요약본)
- Spring Batch Job 구성: Reader, Processor, Writer
- `SlotIndexCalculator`, `PostureStatsWriter` 등 커스텀 구성

**동작 방식**
1. Job은 Scheduler 또는 외부 트리거에 의해 30분 단위로 실행됩니다.
2. 각 Job은 `start_at`, `end_at` 시간을 기준으로 `raw_postures`에서 데이터를 조회합니다.
3. 조회된 데이터를 Slot index(하루를 48개 구간으로 나눈 값)에 따라 그룹화합니다.
4. 각 그룹에 대해 평균 좌표값(avgx, avgy, avgz), 사용자 ID, 시작 시간, Slot 정보를 계산한 뒤 `posture_stats`에 저장합니다.

**기술적 고려 사항**
- Reader는 Cursor 기반 페이징 처리로 메모리 부담을 최소화
- Processor는 단일 사용자-슬롯 단위로 통계 객체 생성
- Writer는 벌크 insert를 위해 `JdbcBatchItemWriter` 사용 가능
- 데이터 중복 방지를 위한 unique constraint or idempotent 설계 필요

---

### 4. Firebase 기반 푸시 알림 발송

**목적**  
사용자의 자세가 일정 시간 동안 개선되지 않거나, 특정 조건을 만족할 때 적절한 피드백을 제공하기 위해 푸시 알림을 전송합니다.

**구성 요소**  
- Firebase Admin SDK
- `FirebaseMessageService`
- 사용자별 FCM 토큰 저장 테이블

**동작 방식**
1. 배치 Job이 통계 계산 후, 특정 Slot에서 장시간 동일 자세가 반복되었는지를 판단합니다.
2. 조건을 만족하는 사용자에게 푸시 알림을 전송하도록 `FirebaseMessageService`를 호출합니다.
3. FCM은 해당 사용자의 기기에 알림을 전송합니다.

**기술적 고려 사항**
- 푸시 알림 전송 조건은 전략적으로 조정 가능 (예: 동일 자세 유지 시간 > 2시간 등)
- 전송 실패 시 재시도 전략 또는 로그 저장 필수
- 사용자별 알림 수신 여부 설정을 고려한 기능 확장 가능

---

### 5. CI/CD 및 배포 구성

**목적**  
코드 변경 사항이 빠르게 서버에 반영되도록 자동화된 빌드 및 배포 파이프라인을 구성합니다.

**구성 요소**
- Jenkins + GitLab
- Dockerfile, docker-compose.yml
- EC2 배포용 SSH 스크립트

**동작 방식**
1. GitLab에 코드 푸시 시 Jenkins가 빌드 트리거
2. `./gradlew build` 후 테스트 및 Docker 이미지 생성
3. Docker Registry에 푸시
4. EC2 서버에서 기존 컨테이너를 중단하고 새 컨테이너를 실행

**기술적 고려 사항**
- `.env` 기반 환경 변수 주입
- 다중 데이터소스(`main`, `batch`) 설정 반영
- 배포 시 `SPRING_PROFILES_ACTIVE` 환경값 지정