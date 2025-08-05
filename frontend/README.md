
# SISEON: 스마트 모니터암 Flutter 앱  

스마트 모니터암 제어를 위한 **Flutter 기반 모바일 애플리케이션**입니다.  
시선 및 자세 기반 모니터 자동 제어, 프리셋 관리, 수동 제어 기능을 제공합니다.

---

## 주요 기능 (Frontend)
### 1. 로그인 및 프로필 관리
- Google OAuth 로그인
- **JWT 인증 기반 토큰 관리**
- 프로필 생성 (아바타, 이름, 키, 시력, 생년월일 입력)
- 로컬 캐싱 (SharedPreferences) 기반 프로필 저장 및 자동 로그인

### 2. 모니터암 제어
- **BLE 통신**: 모터 제어를 위한 실시간 데이터 송신 (수동 모드)
- **MQTT 통신 (TLS 인증)**: Auto/Manual/Preset 모드 전환
- 조이스틱 UI(`flutter_joystick`)를 통한 직관적인 X/Y/Z 제어

### 3. 프리셋 관리
- 서버 연동을 통한 프리셋 저장 및 불러오기
- 프리셋 이름 변경 및 삭제 가능
- 저장된 눈 위치 좌표를 기반으로 모니터 자동 조정

### 4. 알림 및 상태 관리
- **Firebase Cloud Messaging (FCM)** 푸시 알림 (앱 백그라운드/종료 시 알림 수신)
- 현재 제어 모드 및 BLE 연결 상태 실시간 표시

### 5. UI/UX
- Splash → 로그인 → 프로필 선택 → 메인(홈/수동/통계/설정) 화면 구성
- 다크톤 메인 UI (#3B82F6, #000000, #FFFFFF)
- Pretendard 폰트 사용
- 가로모드 자동 전환 (Manual 모드 시)

---

##  기술 스택 (Frontend)

### Flutter & Libraries
- **Flutter SDK**: 3.8.1
- **State Management**: `setState` (리팩토링 시 Provider/Bloc 고려 가능)
- **BLE 통신**: [`flutter_blue_plus`](https://pub.dev/packages/flutter_blue_plus), [`flutter_ble_peripheral`](https://pub.dev/packages/flutter_ble_peripheral)
- **MQTT 통신**: [`mqtt_client`](https://pub.dev/packages/mqtt_client) (TLS 인증 적용)
- **Google OAuth**: `google_sign_in`
- **JWT 인증**: [`jwt_decoder`](https://pub.dev/packages/jwt_decoder)
- **HTTP 통신**: `http`
- **환경 변수 관리**: `flutter_dotenv`
- **로컬 저장소**: `shared_preferences`
- **날짜/로케일**: `intl`, `flutter_localizations`
- **권한 관리**: `permission_handler`
- **FCM 푸시 알림**: `firebase_core`, `firebase_messaging`
- **UI/UX**: `flutter_spinkit`, `cupertino_icons`
- **JSON 직렬화**: `json_annotation`, `json_serializable`, `build_runner`

---

##  폴더 구조 (현재)
```
lib/
├── components/
│   └── custom_joystick.dart
│
├── models/
│   └── control_mode.dart
│
├── pages/
│   ├── settings/
│   │   ├── edit_profile.dart
│   │   ├── preset_page.dart
│   │   ├── profile_page.dart
│   │   └── update_page.dart
│   ├── ble_scan_screen.dart
│   ├── home_screen.dart
│   ├── manual_page.dart
│   ├── profile_create.dart
│   └── stats_page.dart
│
├── services/
│   ├── auth_service.dart
│   ├── fcm_service.dart
│   ├── mqtt_service.dart
│   ├── preset_service.dart
│   └── profile_cache_service.dart
│
├── widgets/
│   └── pretty_joystick.dart
│
├── login_screen.dart
├── main.dart
├── profile_select_screen.dart
└── root_screen.dart
```
> ⚠️ **폴더 구조는 추후 리팩토링 예정**

---

##  구현 내역 (프론트엔드)
- [x] Google OAuth 로그인 및 JWT 기반 인증
- [x] Splash 화면 (2초) 및 Pretendard 폰트 적용
- [x] 프로필 생성 및 선택 UI
- [x] BLE 연결 및 디바이스 스캔/연결 UI
- [x] 조이스틱 기반 수동 제어 (X/Z 축 이동 및 Y 슬라이더)
- [x] MQTT TLS 인증 연결 및 모드 변경 (Auto/Preset/Manual)
- [x] 프리셋 저장, 불러오기, 이름 수정 기능
- [x] Firebase Cloud Messaging (알림 수신)
- [ ] 자세 감지 민감도 설정 UI (추가 예정)
- [ ] OTA 업데이트 화면 구성 (추가 예정)

---

##  실행 방법
### 1. 환경 변수 설정
- 루트 디렉토리에 `.env` 파일 생성
```env
MQTT_BROKER=i13b101.p.ssafy.io
MQTT_PORT=8883
MQTT_CERT=assets/certs/cert.pem
BACKEND_URL=https://api.siseon.com
```

### 2. 의존성 설치 및 실행
```bash
flutter pub get
flutter run
```

---

##  보안
- MQTT TLS 인증서 기반 통신
- JWT 기반 사용자 인증
- Firebase 앱 등록 및 Google OAuth 보안 설정
- 로컬 저장 시 민감 정보 최소화

---

##  진행 현황 (프론트엔드)
- **1주차**: UI 설계, 로그인/프로필 생성 및 캐싱 기능 완료  
- **2주차**: 수동 제어(조이스틱 UI) 구현, 전반적인 UI/UX 개선
- **3주차**: MQTT TLS 인증, 모드 전환 UI 및 상태 관리 추가  
- **4주차 예정**: AI 모델 연동 후 실시간 제어 화면 개선  
