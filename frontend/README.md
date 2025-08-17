# SISEON: 스마트 모니터암 Flutter 앱 (최신 README)

스마트 모니터암 시선(SISEON)을 제어하는 Flutter 기반 모바일 앱입니다. 시선·자세 기반 자동 제어, 프리셋, 수동 제어, 통계/알림/챗봇 등 **앱 서비스 중심**으로 정리했습니다.

---

## 1) 핵심 요약

* **플랫폼**: Flutter (Android 우선)
* **주요 기능**: Google 로그인, 프로필/다계정, BLE 스캔/연결, **Auto/Manual/Preset** 제어, 프리셋 저장·로드·이름변경, FCM 푸시, 통계 시각화, 기본 챗봇
* **통신**: BLE(실시간 제어), \*\*MQTT(TLS)\*\*로 모드 전환, **REST API**로 프로필·프리셋·통계 연동
* **디자인**: 다크톤 + 포인트 컬러 **#3B82F6**, 폰트 **Pretendard**

---

## 2) 화면 플로우

1. **Splash** → 2초 표시
2. **로그인**(Google) → **JWT 수신 및 보관**
3. **프로필 선택/생성**(아바타/이름/키/시력/생년월일)
4. **메인 탭**

   * **홈**: 모드 상태, BLE/디바이스 상태, 요약 정보
   * **수동**: 조이스틱(X/Y/Z) 제어, 가로모드, BLE 전송
   * **챗봇**: 앱 사용 안내/FAQ (기본 가이드)
   * **설정**: 프로필/프리셋/디바이스/알림/업데이트
5. **통계 페이지**: 일·주·월 단위 자세/사용량 시각화, 상세 타임라인(일별)

> ※ 수동 모드에서 가로모드 자동 전환, 조이스틱은 `flutter_joystick` 기반.

---

## 3) 기능 상세

### A. 인증 & 프로필

* **Google Sign-In** → 백엔드 JWT 발급 → 앱 저장
* **프로필 관리**: 생성/수정/선택, 로컬 캐싱(SharedPreferences)
* **다계정**: 프로필별 데이터(디바이스, 통계, 알림 토큰) 분리

### B. 디바이스 & BLE

* **BLE 스캔/연결**: `flutter_blue_plus` 사용
* **특성 쓰기(Characteristic Write)** 기반 **실시간 제어 페이로드** 전송
* **디바이스 등록 흐름**: 서버 조회 → **프로필별 시리얼 캐싱** → 수동/자동 제어 연동

### C. 제어 모드 & MQTT

* 모드: **Auto / Manual / Preset**
* **MQTT(TLS)** 연결: 브로커/포트/CA 인증서 설정, 앱 기동 시 연결 유지
* **토픽 예시**: `/control_mode/1` (팀 설정값 기준)
* 상태 표시: 현재 모드, 연결 상태, 오류 토스트

### D. 프리셋

* **저장/로드/이름변경/삭제**
* 서버와 동기화, **눈 위치 좌표** 기반 모니터 자동 위치 조정

### E. 통계(Stats)

* **posture-stats API**

  * 요청: `GET /posture-stats?profileId=...&period=daily|weekly|monthly`
  * 응답 필드: `avgx, avgy, avgz, createdAt, durationSeconds, startAt, endAt, slotIndex`
* **시각화**: `fl_chart`

  * 일별: **1분 단위 배치**(슬롯) → 히트맵/바/라인 구성
  * 월별: **하루 단위 집계**를 사용(일별 슬롯 누적 결과), 주별도 동일 컨셉
* **나쁜자세 상세**: badReasons 기반 교정 팁/인체공학 팁 카드화
* **타임라인**: 하루 동안 **시간대별 자세 상태 스트립**(가로 타임라인) 구성

### F. 알림(FCM)

* 포그라운드/백그라운드/종료 상태 알림 수신
* **프로필 단위 토큰 등록/삭제**

  * 프로필 선택 시: `POST /api/push/register?profileId=$pid&fcmToken`
  * 로그아웃 시: `POST /api/push/unregister?profileId`
* 알림 탭/딥링크: 메시지 data 기반 라우팅 처리

### G. 챗봇(기본)

* FAQ / 앱 가이드 제공

---

## 4) 아키텍처 개요

* **모바일 앱(Flutter)** ↔ **MQTT Broker**(TLS) ↔ **Raspberry Pi 5**(제어) ↔ **모터/센서**
* **Jetson Orin Nano**(시선/자세 추론) → 백엔드로 요약/상태 전송
* **백엔드(Spring Boot on EC2)** + **RDS**: 프로필/프리셋/통계/알림 토큰 관리
* **Firebase**: Google 로그인/FCM

---

## 5) 기술 스택

### Flutter & 주요 패키지

* **코어**: flutter, intl, http, shared\_preferences
* **IO/연결**: flutter\_blue\_plus, mqtt\_client, permission\_handler, flutter\_foreground\_task
* **UI/UX & 차트**: fl\_chart, flutter\_spinkit, flutter\_joystick, cupertino\_icons
* **인증**: google\_sign\_in, jwt\_decoder
* **설정/유틸**: flutter\_dotenv
* **알림**: firebase\_core, firebase\_messaging

> 각 패키지 **정확한 버전은 `pubspec.yaml`을 기준**으로 합니다.

---

## 6) 폴더 구조(최종)

```
lib/
├─ pages/
│  ├─ settings/
│  │  ├─ preset_page.dart
│  │  ├─ stats_page.dart
│  │  ├─ edit_profile.dart
│  │  ├─ device_info.dart
│  │  └─ settings.page.dart
│  │
│  ├─ ble_scan_screen.dart
│  ├─ device_register_page.dart
│  ├─ manual_page.dart
│  ├─ home_screen.dart
│  ├─ daily_hour_detail_page.dart
│  ├─ profile_create.dart
│  ├─ tutorial_screen.dart
│  └─ chatbot_page.dart
│
├─ services/
│  ├─ auth_service.dart
│  ├─ ble_session.dart
│  ├─ chat_api.dart
│  ├─ device_cache_service.dart
│  ├─ faq_servce.dart
│  ├─ fcm_service.dart
│  ├─ mqtt_service.dart
│  ├─ preset_service.dart
│  ├─ profile_cache_service.dart
│  ├─ snack_service.dart
│  └─ stats_service.dart
│
├─ models/
│  ├─ control_mode.dart
│  ├─ chat_model.dart
│  └─ slot_data.dart
│
├─ widgets/
│  └─ rect_card.dart
│
├─ main.dart
├─ login_screen.dart
├─ root_screen.dart
└─ profile_select_screen.dart
```

---

## 7) 환경 변수(.env) 예시

> 루트에 `.env` 생성 후, `flutter_dotenv`로 로딩

```env
# REST API
API_BASE_URL=https://i13b101.p.ssafy.io/siseon

# MQTT (TLS)
MQTT_HOST=i13b101.p.ssafy.io
MQTT_PORT=8883
MQTT_CONTROL_TOPIC=/control_mode/1
MQTT_CA_CERT=assets/certs/cert.pem

# FCM (Firebase 프로젝트 연동 필요)
FCM_ENABLED=true
```

### pubspec.yaml (assets/폰트)

```yaml

flutter:
  uses-material-design: true

  assets:
    - assets/images/
    - assets/certs/cert.pem
    - .env
  fonts:
    - family: Pretendard
      fonts:
        - asset: assets/fonts/Pretendard-Thin.ttf
          weight: 100
        - asset: assets/fonts/Pretendard-ExtraLight.ttf
          weight: 200
        - asset: assets/fonts/Pretendard-Light.ttf
          weight: 300
        - asset: assets/fonts/Pretendard-Regular.ttf
          weight: 400
        - asset: assets/fonts/Pretendard-Medium.ttf
          weight: 500
        - asset: assets/fonts/Pretendard-SemiBold.ttf
          weight: 600
        - asset: assets/fonts/Pretendard-Bold.ttf
          weight: 700
        - asset: assets/fonts/Pretendard-ExtraBold.ttf
          weight: 800
        - asset: assets/fonts/Pretendard-Black.ttf
          weight: 900
```

### Android 권한/설정(AndroidManifest.xml)

```xml
<uses-permission android:name="android.permission.BLUETOOTH" />
<uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />
<uses-permission android:name="android.permission.BLUETOOTH_SCAN" />
<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
<uses-permission android:name="android.permission.INTERNET" />
<uses-permission android:name="android.permission.WAKE_LOCK" />
<uses-permission android:name="android.permission.FOREGROUND_SERVICE" />
```

> Android 12+에서는 BLUETOOTH\_\* 권한 런타임 요청 필요.

### Firebase(Android)

* **google-services.json**를 `android/app/`에 배치
* **SHA-1/SHA-256**(릴리스 키스토어)을 Firebase 콘솔에 등록 (Google 로그인/FCM 안정화)

  ```bash
  keytool -list -v -keystore release.keystore -alias <ALIAS>
  # → SHA1, SHA-256 값을 콘솔에 추가
  ```

---

## 8) 빌드 & 실행

```bash
# 의존성 설치
flutter pub get

# 디버그 실행
flutter run

# 릴리스 빌드(apk)
flutter build apk --release
```

---

## 9) UI 가이드

* **컬러**: #3B82F6 / #000000 / #FFFFFF
* **폰트**: Pretendard
* **컴포넌트**: 조이스틱 카드, 상태 배지(모드/연결), 통계 카드(일/주/월), 타임라인 스트립

---

## 10) 라이선스/크레딧

* 내부 과제/데모 프로젝트로 사용
* 오픈소스 라이브러리 라이선스는 각 패키지 참조

---

## 11) 변경 이력(요약)

* **2025-08**: 통계 API 연동 정리(일/월 집계 구분), 프로필별 FCM 토큰 처리 명시, MQTT 토픽 예시 명시, 폴더 구조 최신화
* **2025-07**: MQTT TLS, 프로필/디바이스 캐싱 구조 정리, 수동 모드 UX 개선
* **2025-06**: 프리셋/프로필/로그인 초기 구현, FCM 기본 처리
