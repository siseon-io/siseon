// src/pairing_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdio>
#include <array>
#include <string>
#include <set>
#include <chrono>
#include <cstdlib>
#include <unistd.h>

#include <sdbus-c++/sdbus-c++.h>
#include <sdbus-c++/ConvenienceApiClasses.h>

#include <mosquitto.h>

class PairingNode : public rclcpp::Node {
public:
  PairingNode()
  : Node("pairing_node")
  , pairingRequested_(false)
  {
    // 1) ROS2 퍼블리셔
    pub_ = this->create_publisher<std_msgs::msg::String>("/mac_addr", 10);

    // 2) D-Bus Agent 등록
    conn_ = sdbus::createSystemBusConnection();
    registerAgent();

    // 3) MQTT 파라미터
    this->declare_parameter<std::string>("mqtt_host", "i13b101.p.ssafy.io");
    this->declare_parameter<int>("mqtt_port", 1883);
    this->declare_parameter<std::string>("mqtt_user", "khs_pairing");
    this->declare_parameter<std::string>("mqtt_pass", "root1234");
    this->get_parameter("mqtt_host", mqttHost_);
    this->get_parameter("mqtt_port", mqttPort_);
    this->get_parameter("mqtt_user", mqttUser_);
    this->get_parameter("mqtt_pass", mqttPass_);

    // 4) MQTT 초기화 (인증정보 포함)
    mosquitto_lib_init();
    mosq_ = mosquitto_new(nullptr, true, this);
    if (!mqttUser_.empty() || !mqttPass_.empty()) {
      mosquitto_username_pw_set(mosq_,
        mqttUser_.c_str(), mqttPass_.c_str()
      );
    }
    int rc = mosquitto_connect(mosq_,
      mqttHost_.c_str(), mqttPort_, 60
    );
    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
        "❌ MQTT connect() 실패: %s", mosquitto_strerror(rc));
    } else {
      RCLCPP_INFO(get_logger(), "✅ MQTT connect() 성공");
      mosquitto_subscribe(mosq_, nullptr, "request_pair", 0);
      mosquitto_message_callback_set(mosq_, &PairingNode::onMqttMessage);
      mosquitto_loop_start(mosq_);
    }

    // 5) poll 타이머 (request_pair 전까지 무시)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PairingNode::pollConnections, this)
    );
  }

  ~PairingNode() override {
    mosquitto_loop_stop(mosq_, true);
    mosquitto_destroy(mosq_);
    mosquitto_lib_cleanup();
  }

private:
  // Just-Works Agent 등록
  void registerAgent() {
    auto agent = sdbus::createObject(*conn_, "/pairing_agent");
    agent->registerMethod("Release")
         .onInterface("org.bluez.Agent1")
         .implementedAs([](){});
    agent->registerMethod("Cancel")
         .onInterface("org.bluez.Agent1")
         .implementedAs([](){});
    agent->finishRegistration();

    auto mgr = sdbus::createProxy(*conn_, "org.bluez", "/org/bluez");
    mgr->callMethod("RegisterAgent")
       .onInterface("org.bluez.AgentManager1")
       .withArguments(sdbus::ObjectPath("/pairing_agent"),
                      std::string("NoInputNoOutput"));
    mgr->callMethod("RequestDefaultAgent")
       .onInterface("org.bluez.AgentManager1")
       .withArguments(sdbus::ObjectPath("/pairing_agent"));
    RCLCPP_INFO(get_logger(), "Just-Works 에이전트 등록 완료");
  }

  // MQTT 콜백: request_pair 오면 블루투스 활성화
  static void onMqttMessage(struct mosquitto*, void* obj,
                            const struct mosquitto_message* msg)
  {
    auto self = static_cast<PairingNode*>(obj);
    std::string topic(msg->topic);
    RCLCPP_INFO(self->get_logger(),
      "MQTT 수신: topic=%s payload=%.*s",
      topic.c_str(),
      msg->payloadlen,
      reinterpret_cast<const char*>(msg->payload)
    );

    if (topic == "request_pair" && !self->pairingRequested_) {
      RCLCPP_INFO(self->get_logger(), "페어링 요청 수신 → 블루투스 활성화");
      self->startBluetooth();
      self->seen_.clear();
      self->pairingRequested_ = true;

      // 30초 후에 자동으로 광고/발견 모드 끄기
      self->advertiseTimer_ = self->create_wall_timer(
        std::chrono::seconds(30),
        [self]() {
          self->stopBluetooth();
          self->pairingRequested_ = false;
          self->advertiseTimer_->cancel();
        }
      );
    }
  }

  // BLE 켜기
  void startBluetooth()
  {
    // Experimental ON & 데몬 재시작
    std::system(
      "sudo sed -i -E "
      "'s@^#?\\s*Experimental\\s*=.*@Experimental = true@' "
      "/etc/bluetooth/main.conf"
    );
    std::system("sudo systemctl restart bluetooth");

    // LE/Connectable/Bondable/SSP ON
    std::system("sudo btmgmt -i hci0 le on");
    std::system("sudo btmgmt -i hci0 connectable on");
    std::system("sudo btmgmt -i hci0 bondable on");
    std::system("sudo btmgmt -i hci0 ssp on");

    // Advertising 시작
    std::system("sudo hciconfig hci0 up");
    std::system("sudo hciconfig hci0 name pi5");
    std::system("sudo hciconfig hci0 leadv 0");

    // D-Bus 어댑터 경로 및 프록시
    adapterPath_  = "/org/bluez/hci0";
    adapterProxy_ = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);

    RCLCPP_INFO(get_logger(),
      "Bluetooth 활성화 완료: Experimental/LE/Advertising ON");
  }

  // BLE 끄기
  void stopBluetooth()
  {
    std::system("sudo hciconfig hci0 noleadv");
    RCLCPP_INFO(get_logger(), "Bluetooth 광고/발견 모드 비활성화");
  }

  // 주기적으로 hcitool con → Pair/Connect → JSON 발행
  void pollConnections()
  {
    if (!pairingRequested_) return;

    constexpr char CMD[] =
      "hcitool con | grep 'ACL' | awk '{print $3}'";
    FILE* pipe = popen(CMD, "r");
    if (!pipe) {
      RCLCPP_ERROR(get_logger(), "popen() 실패");
      return;
    }

    std::array<char,128> buf;
    while (fgets(buf.data(), buf.size(), pipe)) {
      std::string mac(buf.data());
      mac.erase(mac.find_last_not_of(" \n\r\t")+1);
      if (mac.empty()) continue;

      if (!seen_.count(mac)) {
        seen_.insert(mac);

        // D-Bus device 경로 생성
        std::string path = adapterPath_ + "/dev_";
        for (char c: mac) path += (c==':') ? '_' : c;
        auto dev = sdbus::createProxy(*conn_, "org.bluez", path);

        // 1) Paired 확인
        bool paired = false;
        try {
          paired = dev->getProperty("Paired")
                       .onInterface("org.bluez.Device1");
        } catch(...) {}

        // 2) Pair (필요 시)
        if (!paired) {
          try {
            dev->callMethod("Pair")
               .onInterface("org.bluez.Device1");
            RCLCPP_INFO(get_logger(), "Pair() 성공: %s", mac.c_str());
          } catch(const sdbus::Error &e) {
            RCLCPP_WARN(get_logger(),
              "Pair() 실패(%s), 장치 삭제: %s",
              e.getName().c_str(), mac.c_str());
            adapterProxy_->callMethod("RemoveDevice")
                          .onInterface("org.bluez.Adapter1")
                          .withArguments(sdbus::ObjectPath(path));
            seen_.erase(mac);
            pclose(pipe);
            return;
          }
        }

        // 3) Connect
        try {
          dev->callMethod("Connect")
             .onInterface("org.bluez.Device1");
          RCLCPP_INFO(get_logger(), "Connect() 성공: %s", mac.c_str());
        } catch(const sdbus::Error &e) {
          RCLCPP_WARN(get_logger(),
            "Connect() 실패(%s), 장치 삭제: %s",
            e.getName().c_str(), mac.c_str());
          adapterProxy_->callMethod("RemoveDevice")
                        .onInterface("org.bluez.Adapter1")
                        .withArguments(sdbus::ObjectPath(path));
          seen_.erase(mac);
          pclose(pipe);
          return;
        }

        // 4) JSON 발행
        std::string json = "{\"mac\":\"" + mac + "\"}";
        std_msgs::msg::String msg; msg.data = json;
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "JSON published: %s", json.c_str());

        break;  // 요청 처리 완료 후 루프 종료
      }
    }

    pclose(pipe);
  }

  // 멤버 변수
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr                   timer_;
  rclcpp::TimerBase::SharedPtr                   advertiseTimer_;
  bool                                           pairingRequested_;
  std::set<std::string>                          seen_;
  std::unique_ptr<sdbus::IConnection>            conn_;
  std::unique_ptr<sdbus::IProxy>                 adapterProxy_;
  std::string                                    adapterPath_;
  std::string                                    mqttHost_, mqttUser_, mqttPass_;
  int                                            mqttPort_;
  struct mosquitto*                              mosq_{nullptr};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PairingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
