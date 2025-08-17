# IoT: Pi5 제어 모듈 및 펌웨어

Raspberry Pi 5 기반의 제어 모듈 및 펑웨어 소스코드를 포함한 디렉토리입니다.
5축 로봇 팔을 **실시간 제어**하며, Jetson AI 모듈과 Spring API 서버와 연동하여 사용자의 자세에 맞는 **인체공학적 디스플레이 위치 자동 조정 기능**을 수행합니다.

> **ROS2 기반 분산 제어 시스템으로 구성**

---

## 🔧 아키텍체

### ROS2 노드 구성

```
ros2_ws/src/
├── control_bridge_node/     # MQTT-ROS2 브리지 (제어 모드 수신)
├── fusion_node/             # 센서 데이터 융합 및 자세 결정 
├── arm_control_node/        # 모니터 암 제어 (최종 명령 실행)
├── eye_pose_node/           # 눈 위치 추적 (UDP 소켓으로 Jetson 데이터 수신)
├── preset_bridge_node/      # 프리셋 관리 (MQTT 연동)
├── manual_bt_node/          # 수동 블루투스 제어 
└── pairing_bridge_node/     # 디바이스 페어링 (MQTT 연동)
```

### 통신 방식

* **MQTT**: Flutter, Spring ↔ Pi5 (SSL/TLS 보안 통신)
* **ROS2 Topics**: 노드 간 실시간 데이터 교환
* **UDP Socket**: Jetson ↔ Pi5

---

## ⚙️ 설치 및 실행

### 1. 시스템 요구사항

* OS: Ubuntu 24.04 LTS
* ROS2: Jazzy
* HW: Raspberry Pi 5 (RAM 4GB 이상)

### 2. 의존성 설치

```bash
# ROS2 Jazzy 설치
sudo apt update && sudo apt install ros-jazzy-desktop

# 추가 패키지
sudo apt install -y \
  ros-jazzy-ament-cmake \
  ros-jazzy-rclcpp \
  ros-jazzy-geometry-msgs \
  ros-jazzy-std-msgs \
  libnlohmann-json3-dev \
  libpaho-mqtt-dev \
  libpaho-mqttpp-dev
```

### 3. 빌드 및 실행

```bash
cd iot/ros2_ws
bash run_all.sh
```

#### 환경변수 설정

.env 또는 시스템 환경변수 대입:

```bash
export MQTT_HOST="your-mqtt-broker.com"
export MQTT_PORT="8883"
export MQTT_PROTOCOL="mqtts"
export DEVICE_ID="pi5_device_001"
export MQTT_USER="username"
export MQTT_PASSWD="password"
```

#### 노드 개별 실행

```bash
ros2 run control_bridge_node control_bridge_node
```

---

## 📡️ 네트워크 및 보안

* **MQTT Broker**: EMQX
* **보안 연결**: TLS 인증서 기반 SSL 통신
* **Jetson ↔ Pi5**: UDP Socket
* **방화법**: Pi5는 내년망에서만 접근 가능하도록 해설

---

## 🔌 하드웨어 인터피스

* Jetson ↔ Pi5: UDP Socket 시리얼
* Pi5 ↔ Arm: GPIO (PWM, Relay), I2C (PCA9685)
* 모터:
  * XM430-W350-T 다이나믹셀
* 센서:
  * YDLIDAR X4 PRO (270도 범위)
  * RGB 카메라

---

## 🔗 소프트웨어 인터피스

### MQTT Topics

| 토픽명                           | 발행자              | 구독자              | 설명                    |
| ------------------------------- | ------------------ | ------------------ | ---------------------- |
| `/control_mode/{device_id}`     | Spring/Flutter     | control_bridge_node | 제어 모드 수신 (auto/manual/preset) |
| `/request_pair/{device_id}`     | Spring/Flutter |   pairing_bridge_node   | 디바이스 페어링 요청       |
| `/preset_coordinate/{device_id}` | Spring/Flutter     | preset_bridge_node | 프리셋 좌표 저장/로드 요청  |

### /request_pair/{device_id}

```json
{
  "profile_id": string
}
```

### /preset_coordinate/{device_id}

```json
{
	"profileId": string
  "lefteyeX": float,
  "lefteyeY": float,
  "lefteyeZ": float,
  "righteyeX": float,
  "righteyeY": float,
  "righteyeZ": float,
}
```

### /control_mode/{device_id}

```json
{
  "profile_id": string
  "previous_mode": sring
  "current_mode": string
}
```

### ROS2 Topics

| 토픽명          | 발행자 노드          | 구독자 노드         | 설명                        |
| -------------- | ------------------ | ------------------ | --------------------------- |
| `/eye_pose`    | eye_pose_node      | fusion_node        | Jetson에서 추적된 사용자 눈 위치 (x, y, z) |
| `/mac_addr`    | pairing_bridge_node | manual_bt_node     | 페어링된 디바이스 MAC 주소        |
| `/manual_pose` | manual_bt_node     | fusion_node        | 수동 블루투스 제어로 설정된 자세     |
| `/preset_pose` | preset_bridge_node | fusion_node        | 프리셋으로 저장된 자세 데이터       |
| `/control_mode`| control_bridge_node | fusion_node        | 제어 모드 전환 신호            |
| `/cmd_pose`    | fusion_node        | arm_control_node   | 로봇팔 목표 위치 전달 (최종 제어 명령) |

### /eye_pose

```json
{
  "lefteye_x": float,
  "lefteye_y": float,
  "lefteye_z": float,
  "righteye_x": float,
  "righteye_y": float,
  "righteye_z": float,
}
```

### /manual_pose

```json
{
  "lefteye_x": float,
  "lefteye_y": float,
  "lefteye_z": float,
  "righteye_x": float,
  "righteye_y": float,
  "righteye_z": float,
}
```

### /preset_pose

```json
{
  "lefteye_x": float,
  "lefteye_y": float,
  "lefteye_z": float,
  "righteye_x": float,
  "righteye_y": float,
  "righteye_z": float,
}
```

### /cmd_pose

```json
{
  "m11": float,
  "m12": float,
  "m13": float,
  "m14": float
}
```

### /control_mode

```json
{
  "data": "auto" or "manual" or "preset" or "fix" or "off"
}
```