## 디렉터리 & 패키지 구조 예시(수정될 수 있음)
```
📁 siseon-service/
├── 📄 build.gradle.kts
├── 📄 Jenkinsfile
├── 📁 src
│   ├── 📁 main
│   │   ├── 📁 java
│   │   │   └── 📦 siseon.service
│   │   │       ├── 📄 SiseonServiceApplication.java
│   │   │       ├── 📁 config        # 보안·CORS·HTTP 클라이언트 설정
│   │   │       ├── 📁 domain        # JPA 엔티티 (Profile, Device, User, Preset, Session, PoseData, EyeData)
│   │   │       ├── 📁 repository    # JpaRepository 인터페이스
│   │   │       ├── 📁 dto           # Request/Response DTO
│   │   │       ├── 📁 service       # 비즈니스 로직 (Auth, Preset, Dashboard, PiControl)
│   │   │       ├── 📁 controller    # REST API 컨트롤러
│   │   │       └── 📁 security      # JWT·OAuth2 필터 및 설정
│   │   └── 📁 resources
│   │       ├── 📄 application.yml
│   │       ├── 📄 application-dev.yml
│   │       └── 📄 application-prod.yml
│   └── 📁 test
│       └── 📦 siseon.service      # 단위·통합 테스트
```


## 1. 개발 환경 준비  
- Java 17 설치 및 `JAVA_HOME`/`PATH` 설정  
- Gradle Wrapper 생성(`gradle wrapper --gradle-version 8.x`) 및 실행 확인  
- IDE(IntelliJ/VSCode)에서 Gradle 프로젝트로 Import, SDK 17 지정  



## 2. 프로파일 & 설정 분리  
- **application.yml** (공통)  
  - RDS 접속 정보 (`spring.datasource.*`)  
  - JPA `ddl-auto=update`  
  - Pi5 엣지 서버 주소 (`pi.base-url: http://<PI5_IP>:5000`)  
  - JWT 시크릿·만료시간  
  - CORS 허용 도메인  
- **application-dev.yml** / **application-prod.yml**  
  - 각 환경별 RDS, OAuth2, HTTPS 키스토어 등 설정 분리  


## 3. 데이터베이스 연결  
- AWS RDS(MySQL) 인스턴스 생성 및 보안 그룹(3306) 설정  
- `DB_HOST`, `DB_USER`, `DB_PASS` 환경변수로 주입  
- 애플리케이션 기동 시 엔티티→테이블 자동 동기화 확인 (`SHOW TABLES;`)  


## 4. 도메인 모델 & JPA 리포지토리  
- 엔티티 정의: Profile, Device, User, Preset, Session, PoseData, EyeData  
- 컬럼 제약 및 관계 매핑(`@ManyToOne`, `@OneToMany`)  
- 각 엔티티별 `JpaRepository` 인터페이스 구현  


## 5. DTO & 매핑  
- 인증·프리셋·대시보드용 Request/Response DTO 정의 (`@Valid`, `@NotNull` 등)  
- ModelMapper 또는 MapStruct 빈 등록 및 매핑 프로필 작성  


## 6. 서비스 계층  
- **AuthService**: 회원가입·로그인·JWT 발급  
- **PresetService**: CRUD + 소유자 검증  
- **DashboardService**: 일·주·월별 통계 집계·히스토리 조회  
- **PiControlService**:  
  - `moveArm(x,y,z)` 호출 → Spring 내 `RestTemplate` 또는 `WebClient` 로 `POST http://<PI5_IP>:5000/move-arm`  
  - `applyPreset(id)` 호출 → `POST http://<PI5_IP>:5000/apply-preset`  
  - 실패 시 Spring Retry를 통한 재시도  


## 7. REST API & 엣지 서버 통신  
- **AuthController**: `/api/auth/register`, `/api/auth/login`  
- **PresetController**:  
  - `/api/presets` CRUD  
  - `/api/presets/{id}/move` → `PiControlService.moveArm()`  
- **DashboardController**: `/api/dashboard/stats`, `/api/dashboard/history`  
- **GlobalExceptionHandler**: `@ControllerAdvice` + `@ExceptionHandler` → 일관된 오류 응답  
- **Pi 엣지 서버** (`Flask`):  
  - `POST /move-arm` → `{ x, y, z }` JSON 수신 → 모터 제어  
  - `POST /apply-preset` → `{ presetId }` 수신 → 좌표 조회 → 모터 제어  
  - CORS: Spring 도메인 허용  


## 8. 보안 설정  
- **WebSecurityConfig**:  
  - `/api/auth/**`, `/oauth2/**` 공개, 기타 경로 JWT 인증 필요  
  - OAuth2 Client + JWT Resource Server 설정  
  - CORS 정책으로 Flutter 도메인 허용  
  - 운영 환경 HTTP→HTTPS 리다이렉트  
- **JwtTokenProvider** & **JwtAuthenticationFilter** 구현  


## 9. HTTP 클라이언트 설정  
- `RestTemplate` / `WebClient` Bean 등록 (connection/read timeout 설정)  
- 예외(`RestClientException`, `HttpStatusCodeException`) 핸들링 및 로깅  


## 10. 로깅 & 모니터링  
- Logback 설정(`logback-spring.xml`)으로 파일·콘솔 로그 포맷 정의  
- Spring Actuator 활성화: `/actuator/health`, `/metrics`  


## 11. 테스트  
- 단위 테스트(JUnit 5 + Mockito) 작성  
- 통합 테스트(`@SpringBootTest`, Testcontainers(MySQL) 등) 구성  


## 12. CI/CD (Jenkins)  
- **Jenkinsfile** 단계:  
  1. 코드 체크아웃  
  2. `./gradlew clean build`  
  3. 테스트 실행  
  4. Docker 이미지 빌드 & 태깅  
  5. Docker 레지스트리 푸시  
  6. **EC2 SSH 배포 스크립트 실행** (아래 참조)  
- Jenkins Credential 등록: RDS, Pi5 IP, JWT_SECRET, Docker Registry  

### EC2 배포 스크립트 예시
```bash
ssh ec2-user@<EC2_HOST> << 'EOF'
  docker pull registry.example.com/siseon-service:${BUILD_NUMBER}
  docker stop siseon-service || true
  docker rm siseon-service || true
  docker run -d \
    --name siseon-service \
    -e SPRING_PROFILES_ACTIVE=prod \
    -e DB_HOST=${DB_HOST} \
    -e DB_USER=${DB_USER} \
    -e DB_PASS=${DB_PASS} \
    -e PI_BASE_URL=${PI_BASE_URL} \
    -e JWT_SECRET=${JWT_SECRET} \
    registry.example.com/siseon-service:${BUILD_NUMBER}
EOF
```