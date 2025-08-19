# SISEON 모니터암 제어 백엔드

사용자의 자세 데이터를 주기적으로 수집·분석하고, 저장된 **프리셋**을 모니터암에 적용하여 자동으로 제어하는 IoT 백엔드 시스템

> 수집(HTTP API, 10초) → 원본 저장(`raw_postures`) → **분(minute) 배치** 집계(`posture_stats`) → **일(day) 배치** 집계(`posture_stats_day`) → 프리셋 MQTT 발행(`/preset_coordinate`) → FCM 푸시 알림

<img width="657" height="457" alt="image" src="https://github.com/user-attachments/assets/e770b88b-aa6d-405e-9391-b44f2551f998" />


---

- **수집 경로 **: Raspberry Pi → **HTTP API** 전송(이전: MQTT 수신 아님)
- **배치 주기 **: raw-> batch 1분(minute) / batch -> batch 1일(day) 2중 배치**
- **DB Batch 주기**: 수신된 PoseData를 **10초 간격**으로 `raw_postures`에 저장 후, 배치로 `posture_stats`/`posture_stats_day` 생성
- **MQTT**: 프리셋 적용 시에만 **백엔드 → MQTT 발행**(`/preset_coordinate`)
- **불필요 항목 제거**: Edge Flask 언급 제거

---

## 🌐 시스템 아키텍처 (개요)

```
[ Raspberry Pi ]
   └─ HTTP POST /api/raw-postures (10s interval)
            │
            ▼
[ Spring Boot Backend ]
   ├─ 원본 저장: raw_postures (main 스키마)
   ├─ 분(minute) 배치: posture_stats (batch 스키마)
   ├─ 일(day) 배치: posture_stats_day (batch 스키마)
   ├─ 프리셋 적용: MQTT publish -> /preset_coordinate (QoS 1)
   └─ 푸시 알림: Firebase Cloud Messaging (조건 만족 시)
            │
            ▼
[ Raspberry Pi (모터 제어) ]
   └─ MQTT 구독(/preset_coordinate) → 모니터암 구동
```

---

## 📥 데이터 수집 (HTTP API)

- **주기**: 약 10초 간격(디바이스에서 주기 전송)
- **엔드포인트**: `POST /api/raw-postures`
- **역할**: 디바이스가 측정한 사용자 자세(관절 좌표, 모니터 좌표 등)를 **원본 그대로** `raw_postures` 테이블에 적재  
- **비고**: 저장 단계에서는 판단/알림 로직 수행하지 않음 (집계는 배치 단계에서)

예시 페이로드(축약):
```json
{
  "profileId": 26,
  "monitorCoord": {"x": 0.12, "y": 0.34, "z": 0.56},
  "userCoord": {
    "le_shoulder": {"x": -0.12, "y": 1.02, "z": 0.04},
    "re_shoulder": {"x": 0.23, "y": 1.01, "z": 0.03}
  },
  "capturedAt": "2025-08-17T09:00:10Z"
}
```

---

## 배치 통계 처리

### 1) **분(minute) 배치** → `posture_stats`
- **주기/트리거**: 스케줄러 또는 배치 잡(분 단위)
- **입력**: `raw_postures` (10초 간격 데이터)
- **그룹 기준**: `profileId` × `분(minute)` × (슬롯 인덱스 `slotIndex` 등)
- **출력 필드 예**: `startAt`, `endAt`, `durationSeconds`, `validPosture`, `monitorCoord`, `userCoord`, `profileId`, `slotIndex`
- **용도**: 그래프/알림의 기본 단위, 자세 판정(거북목/굽은 어깨 등) 요약

### 2) **일(day) 배치** → `posture_stats_day`
- **주기**: 1일 1회
- **입력**: `posture_stats`
- **출력**: 일자별 좋은/나쁜 자세 **비율과 횟수**(요청 반영), 총 지속시간 등
- **용도**: 대시보드의 일·주·월 통계, 장기 추이 분석

> 알림(FCM)은 **분 배치 결과**를 기반으로, 일정 기간 **나쁜 자세 패턴**이 지속될 때 조건부로 전송합니다.

---

## 프리셋 적용 (MQTT Publish)

- **토픽**: `/preset_coordinate`
- **QoS**: 1 (At least once)
- **페이로드(예)**:
```json
{
  "profileId": 26,
  "presetId": 7,
  "coordinate": {"x": 0.15, "y": 0.30, "z": 0.70}
}
```
- **흐름**: 사용자가 앱에서 프리셋을 선택 → 백엔드가 프리셋 좌표를 조회 → **MQTT 발행** → Pi가 구독 후 모터 제어 수행

> 주의: **자세 데이터 수집은 MQTT를 사용하지 않습니다.** 수집은 오직 **HTTP API**.

---

## 기술 스택


- **Backend**: Spring Boot 3.5.3, Spring Batch, Spring Data JPA, Spring Security
- **DB**: MySQL 8.0 (스키마 분리: `main` / `batch`)

---

## 구성 요약

| 항목       | 사용 기술/설정 |
|-----------|-----------------|
| 백엔드    | Spring Boot 2.5.5, Spring Batch, JPA |
| DB        | MySQL 8.0, 다중 데이터소스(`main`, `batch`) |
| 메시징    | MQTT (EMQX), **Publish only** `/preset_coordinate`, QoS 1 |
| 푸시알림  | Firebase Cloud Messaging (FCM) |
| 배포      | Docker, Jenkins, AWS EC2 |
| 보안      | JWT 인증, Nginx Reverse Proxy + TLS(ACME/Certbot), OAuth2(Google) |

---

```
📁 siseon-backend/
├── 📄 Dockerfile
├── 📄 docker-compose.yml
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
│       │       │   ├── 📁 main            # 사용자, 프리셋, RawPosture 등 실시간 데이터
│       │       │   └── 📁 batch           # PostureStats, 통계 집계용 엔티티
│       │       ├── 📁 repository
│       │       │   ├── 📁 main            # main DB JPA 리포지토리
│       │       │   └── 📁 batch           # batch DB JPA 리포지토리
│       │       ├── 📁 service             # 비즈니스 로직
│       │       ├── 📁 controller          # REST API 엔드포인트
│       │       ├── 📁 dto                 # Request, Response DTO
│       │       ├── 📁 batch               # Spring Batch Job 구성
│       │       ├── 📁 scheduler           # 배치 스케줄러
│       │       ├── 📁 listener            # Batch Step Listener
│       │       ├── 📁 mqtt                # MQTT 발행/수신 처리
│       │       └── 📁 websocket           # 웹소켓 처리
│       └── 📁 resources
|           └── 📁 firebase
|                └── 📄 serviceAccountKey.json
│           └── 📄 application.yml
└── 📁 test
```

---

- **MQTT 발행 핸들러**: `MqttHeaders.TOPIC`를 발행 시점에 설정하여 `/preset_coordinate`로 송신(QoS 1 권장)
- **배치 안정성**: idempotent 설계(중복 집계 방지), 트랜잭션 경계 명확화, 인덱스/파티셔닝 검토(데이터 증가 대비)
- **프로파일 분리**: 다중 데이터소스 빈 분리(`main`/`batch`)

---
