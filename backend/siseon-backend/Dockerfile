# ┌── 빌드 스테이지: Gradle + JDK17
FROM gradle:7.6-jdk17 AS builder
WORKDIR /home/gradle/project

# 의존성 캐시
COPY build.gradle settings.gradle gradlew ./
COPY gradle gradle

# gradlew 실행 권한 추가
RUN chmod +x ./gradlew

# 의존성 다운로드 및 빌드 준비
RUN ./gradlew --no-daemon clean assemble

# 실제 소스 복사 후 JAR 생성
COPY src src
RUN ./gradlew --no-daemon bootJar -x test

# └── 런타임 스테이지: OpenJDK17 슬림
FROM openjdk:17-jdk-slim
WORKDIR /app

# 빌드 결과물만 복사
COPY --from=builder /home/gradle/project/build/libs/*.jar app.jar

ENTRYPOINT ["java", "-jar", "/app/app.jar"]
