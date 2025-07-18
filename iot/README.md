# IoT: Pi5 제어 모듈 및 펌웨어

## 개요

이 디렉토리는 Raspberry Pi 5 기반의 제어 모듈 및 펌웨어 소스코드를 포함합니다.  
SCARA 방식 3축 모니터 암을 실시간으로 제어하며, Jetson AI 모듈 및 Spring API 서버와 연동하여 인체공학적 디스플레이 위치 자동 조정 기능을 담당합니다.

---

## 주요 기능

- Jetson AI 모듈로부터 실시간 사용자 위치 데이터 수신(UART)
- SCARA 모니터 암(X/Y/Z축) 제어 (PWM, GPIO, I2C)
- 프리셋 적용 및 수동 제어 명령 처리(HTTP REST)
- 상태 모니터링 및 피드백 전송
- 펌웨어 OTA 업데이트 지원(예정)

---

## 설치 및 실행 방법

1. **OS**: VxWorks OS (64-bit) 또는 Raspberry Pi OS
2. **필수 패키지 설치**  
   - GPIO/PWM/I2C 제어 라이브러리
   - HTTP 서버 모듈
3. **빌드 및 실행**
   - C/C++ 소스코드 빌드
   - 서비스 등록 및 자동 실행 설정
4. **네트워크/시리얼 설정**
   - Jetson ↔ Pi5: UART 3.3V TTL, 115200bps
   - Pi5 ↔ EC2: HTTP REST API

---

## 하드웨어 인터페이스

- **Jetson ↔ Pi5**: UART 시리얼 통신
- **Pi5 ↔ 모니터암**: GPIO(PWM, Relay), I2C(PCA9685)
- **서보모터**: MG996R (PWM 제어, 6V)
- **리니어 액추에이터**: 12V 릴레이 구동

---

## 소프트웨어 인터페이스

- **HTTP API**
  - `POST /apply-preset`: 프리셋 적용
  - `POST /move-arm`: 수동 좌표 이동
- **UART 프로토콜**
  - `EYE_POS,x,y,z`: 눈 위치 데이터
  - `RESET`, `RE-CALIBRATE`: 제어 명령

---

## 개발 환경

- **언어**: C/C++
- **테스트**: pytest (시뮬레이터 기반)
- **배포**: 로컬 빌드 후 Pi5에 업로드

---

## 보안 및 주의사항

- Pi5는 내부망에서만 접근 가능하도록 방화벽 설정 권장
- JWT 기반 인증 적용 예정
- 카메라 이미지 등 개인정보 저장 금지

---

## 참고 자료

- [Raspberry Pi 공식 문서](https://www.raspberrypi.com/documentation/)
- [PCA9685 PWM Driver Guide](https://learn.adafruit.com/16-channel-pwm-servo-driver)
- [Jetson UART 통신 가이드](https://developer.nvidia.com/embedded/jetson-linux)

---

## 문의 및 기여

이 모듈 관련 문의, 버그 제보, 기여는 프로젝트 담당자 또는 이슈 트래커를 통해 남겨주세요.
