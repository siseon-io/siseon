// src/pairing_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdlib>
#include <string>
#include <set>
#include <vector>
#include <map>
#include <array>
#include <chrono>
#include <cstdio>
#include <unistd.h>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <sdbus-c++/sdbus-c++.h>
#include <mqtt/async_client.h>

// xml2cpp로 생성된 GATT 서비스·특성 어댑터 헤더
#include "gatt_service_adaptor.h"
#include "gatt_characteristic_adaptor.h"

static constexpr int QOS = 1;
using namespace std::chrono_literals;

// ── GattService 어댑터 ────────────────────────────────────────────────
class GattService : public sdbus::AdaptorInterfaces<org::bluez::GattService1_adaptor>
{
public:
    GattService(sdbus::IConnection& connection, std::string objectPath)
    : AdaptorInterfaces(connection, std::move(objectPath))
    {
        registerAdaptor();
    }
    ~GattService() { unregisterAdaptor(); }

    std::string UUID() override { return "12345678-1234-5678-1234-56789ABCDEF0"; }
    bool Primary() override        { return true; }
};

// ── GattCharacteristic 어댑터 ─────────────────────────────────────────
class GattApplication;  // 전방 선언

class GattCharacteristic : public sdbus::AdaptorInterfaces<org::bluez::GattCharacteristic1_adaptor>
{
public:
    GattCharacteristic(sdbus::IConnection& connection,
                       std::string objectPath,
                       GattApplication* app)
    : AdaptorInterfaces(connection, std::move(objectPath))
    , app_(app)
    {
        registerAdaptor();
    }
    ~GattCharacteristic() { unregisterAdaptor(); }

    std::vector<uint8_t> ReadValue(const std::map<std::string, sdbus::Variant>& options) override;
    void WriteValue(const std::vector<uint8_t>& value,
                    const std::map<std::string, sdbus::Variant>& options) override;

    std::string UUID() override  { return "12345678-1234-5678-1234-56789ABCDEF1"; }
    sdbus::ObjectPath Service() override {
        return sdbus::ObjectPath("/org/example/application/service0");
    }
    std::vector<std::string> Flags() override { return {"read", "write"}; }

private:
    GattApplication* app_;
};

// ── GattApplication: ObjectManager 인터페이스를 편의 API로 등록 ─────────
class GattApplication
{
public:
    GattApplication(sdbus::IConnection& connection, const std::string& objectPath)
    : conn_(connection)
    , objPath_(objectPath)
    {
        // ObjectManager 전체를 등록
        obj_ = sdbus::createObject(conn_, objPath_);
        obj_->addObjectManager();
        obj_->finishRegistration();

        // GATT 서비스/특성 어댑터 생성
        service_        = std::make_unique<GattService>(conn_, objPath_ + "/service0");
        characteristic_ = std::make_unique<GattCharacteristic>(conn_, objPath_ + "/service0/char0", this);
    }

    ~GattApplication() = default;

    // ObjectManager가 호출할 메서드
    std::map<sdbus::ObjectPath,
             std::map<std::string, std::map<std::string, sdbus::Variant>>>
    GetManagedObjects()
    {
        std::map<sdbus::ObjectPath,
                 std::map<std::string, std::map<std::string, sdbus::Variant>>>
            response;

        // Service 등록 정보
        {
            std::map<std::string, sdbus::Variant> props;
            props["UUID"]    = sdbus::Variant(service_->UUID());
            props["Primary"] = sdbus::Variant(service_->Primary());
            response[sdbus::ObjectPath(service_->getObjectPath())]
                    ["org.bluez.GattService1"] = props;
        }

        // Characteristic 등록 정보
        {
            std::map<std::string, sdbus::Variant> props;
            props["UUID"]    = sdbus::Variant(characteristic_->UUID());
            props["Service"] = sdbus::Variant(sdbus::ObjectPath(characteristic_->Service()));
            props["Flags"]   = sdbus::Variant(characteristic_->Flags());
            response[sdbus::ObjectPath(characteristic_->getObjectPath())]
                    ["org.bluez.GattCharacteristic1"] = props;
        }

        return response;
    }

    // Read/Write 시 사용할 초기값
    std::vector<uint8_t> characteristic_value_{ 'H','e','l','l','o' };

private:
    sdbus::IConnection& conn_;
    std::string          objPath_;
    std::unique_ptr<sdbus::IObject>             obj_;
    std::unique_ptr<GattService>                 service_;
    std::unique_ptr<GattCharacteristic>          characteristic_;
};

// ReadValue 구현 (바이트 배열을 16진수 스트링으로 변환해 로그에 출력)
std::vector<uint8_t> GattCharacteristic::ReadValue(
  const std::map<std::string, sdbus::Variant>&)
{
  auto& data = app_->characteristic_value_;
  // 바이트 배열을 2자리 16진수 문자열로 변환
  std::ostringstream oss;
  for (auto b : data) {
      oss << std::hex << std::setw(2) << std::setfill('0') << (int)b << ' ';
  }
  RCLCPP_INFO(rclcpp::get_logger("pairing_node"),
              "ReadValue 호출, 반환 데이터: [%s]",
              oss.str().c_str());

  return data;
}

// WriteValue 구현 (수신한 바이트 배열을 16진수 스트링으로 변환해 로그에 출력)
void GattCharacteristic::WriteValue(
  const std::vector<uint8_t>& value,
  const std::map<std::string, sdbus::Variant>&)
{
  // 수신된 데이터를 16진수 문자열로 변환
  std::ostringstream oss;
  for (auto b : value) {
      oss << std::hex << std::setw(2) << std::setfill('0') << (int)b << ' ';
  }
  RCLCPP_INFO(rclcpp::get_logger("pairing_node"),
              "WriteValue 호출, 수신 데이터: [%s]",
              oss.str().c_str());

  // 내부 값 갱신
  app_->characteristic_value_ = value;
}
// ── PairingNode 클래스 ─────────────────────────────────────────────────
class PairingNode
  : public rclcpp::Node
  , public virtual mqtt::callback
{
public:
  PairingNode()
  : Node("pairing_node")
  {
    // ROS2 퍼블리셔
    pub_ = create_publisher<std_msgs::msg::String>("/mac_addr", 10);

    // D-Bus 시스템 버스 연결 및 이벤트 루프 시작
    conn_ = sdbus::createSystemBusConnection();
    conn_->enterEventLoopAsync();
    adapterPath_ = "/org/bluez/hci0";

    registerAgent();
    registerGattApplication();

    loadEnvironment();
    initMqtt();

    // 주기적으로 연결된 디바이스를 탐색
    timer_ = create_wall_timer(1s, [this]() { pollConnections(); });
  }

  ~PairingNode() override
  {
    try { client_->disconnect()->wait(); } catch(...) {}

    unregisterAdvertisement();

    if (gatt_app_) {
      auto mgr = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
      mgr->callMethod("UnregisterApplication")
         .onInterface("org.bluez.GattManager1")
         .withArguments(sdbus::ObjectPath("/org/example/application"));
      RCLCPP_INFO(get_logger(), "🚫 GATT Application 해제 완료");
    }

    conn_->leaveEventLoop();
  }

  // mqtt::callback 구현
  void connection_lost(const std::string &cause) override {
    RCLCPP_WARN(get_logger(), "MQTT 연결 상실: %s", cause.empty() ? "Unknown" : cause.c_str());
  }

  void message_arrived(mqtt::const_message_ptr msg) override {
    auto topic   = msg->get_topic();
    auto payload = msg->to_string();
    RCLCPP_INFO(get_logger(), "MQTT 수신: topic=%s payload=%s", topic.c_str(), payload.c_str());

    // 페어링 요청이 오면 BLE 광고 시작
    if (topic == topicRequest_ && !pairingRequested_) {
      RCLCPP_INFO(get_logger(), "🛎 페어링 요청 수신 → BLE 광고 시작");
      pairingRequested_ = true;
      seen_.clear();

      makeAdapterDiscoverable();
      registerAdvertisement();

      advertiseTimer_ = create_wall_timer(30s, [this]() {
        unregisterAdvertisement();
        pairingRequested_ = false;
        advertiseTimer_->cancel();
      });
    }
  }

  void delivery_complete(mqtt::delivery_token_ptr) override {
    // 사용하지 않음
  }

private:
  // Just-Works 에이전트 등록
  void registerAgent() {
    auto agent = sdbus::createObject(*conn_, "/pairing_agent");
    agent->registerMethod("Release").onInterface("org.bluez.Agent1").implementedAs([](){});
    agent->registerMethod("Cancel" ).onInterface("org.bluez.Agent1").implementedAs([](){});
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

  // 어댑터 Discoverable/Pairable 모드로 설정
  void makeAdapterDiscoverable() {
    auto adapter = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
    adapter->callMethod("Set")
           .onInterface("org.freedesktop.DBus.Properties")
           .withArguments("org.bluez.Adapter1", "Powered",    sdbus::Variant(true));
    adapter->callMethod("Set")
           .onInterface("org.freedesktop.DBus.Properties")
           .withArguments("org.bluez.Adapter1", "Discoverable",sdbus::Variant(true));
    adapter->callMethod("Set")
           .onInterface("org.freedesktop.DBus.Properties")
           .withArguments("org.bluez.Adapter1", "Pairable",   sdbus::Variant(true));
    RCLCPP_INFO(get_logger(), "🔵 어댑터 Powered/Discoverable/Pairable 설정 완료");
  }

  // 환경변수 조회 헬퍼
  template<typename T>
  void getEnv(const char* key, T &out, bool required, T def = T{}) {
    if (auto v = std::getenv(key)) {
      if constexpr(std::is_same_v<T, std::string>) out = v;
      else out = static_cast<T>(std::stoi(v));
    } else if (required) {
      RCLCPP_FATAL(get_logger(), "%s 환경변수를 설정하세요.", key);
      throw std::runtime_error("Missing " + std::string(key));
    } else {
      out = def;
    }
  }

  // MQTT 설정 정보 로드
  void loadEnvironment() {
    getEnv("MQTT_HOST",    mqttHost_,   true);
    getEnv("MQTT_PORT",    mqttPort_,   true);
    getEnv("DEVICE_ID",    deviceId_,   true);
    getEnv("MQTT_USER",    mqttUser_,   false);
    getEnv("MQTT_PASSWD",  mqttPasswd_, false);
    getEnv("MQTT_CA_CERT", sslCaPath_,  false);
    getEnv("MQTT_PROTOCOL", proto_,     false, std::string("mqtts"));
    getEnv("MQTT_KEEP_ALIVE", keepAlive_, false, 60);
    topicRequest_ = "/request_pair/" + deviceId_;
  }

  // MQTT 초기화 및 구독
  void initMqtt() {
    std::string address =
      (proto_ == "mqtt" || proto_ == "tcp" ? "tcp://" : "ssl://") +
      mqttHost_ + ":" + std::to_string(mqttPort_);
    RCLCPP_INFO(get_logger(), "🔗 MQTT broker: %s", address.c_str());

    client_ = std::make_unique<mqtt::async_client>(address, "pairing_node");
    client_->set_callback(*this);

    auto sslb = mqtt::ssl_options_builder();
    if (!sslCaPath_.empty()) {
      sslb.trust_store(sslCaPath_).enable_server_cert_auth(true);
      RCLCPP_INFO(get_logger(), "🔒 CA cert: %s", sslCaPath_.c_str());
    } else {
      sslb.enable_server_cert_auth(false);
      RCLCPP_WARN(get_logger(), "⚠️ MQTT_CA_CERT 미설정: 서버 인증서 검증 OFF");
    }
    sslOpts_ = sslb.finalize();

    connOpts_ = mqtt::connect_options_builder()
                  .clean_session()
                  .user_name(mqttUser_)
                  .password(mqttPasswd_)
                  .ssl(sslOpts_)
                  .automatic_reconnect(true)
                  .keep_alive_interval(std::chrono::seconds(keepAlive_))
                  .finalize();

    try {
      RCLCPP_INFO(get_logger(), "🔄 MQTT 연결 시도…");
      client_->connect(connOpts_)->wait();
      RCLCPP_INFO(get_logger(), "✅ MQTT 연결 성공");
      client_->subscribe(topicRequest_, QOS)->wait();
      RCLCPP_INFO(get_logger(), "📡 구독: %s", topicRequest_.c_str());
    } catch (const mqtt::exception &e) {
      RCLCPP_ERROR(get_logger(), "❌ MQTT 연결/구독 실패: %s", e.what());
      throw;
    }
  }

  // BLE 광고 등록
  void registerAdvertisement() {
    if (advObj_) return;

    advObj_ = sdbus::createObject(*conn_, "/pairing_advertisement");
    advObj_->registerMethod("Release")
           .onInterface("org.bluez.LEAdvertisement1")
           .implementedAs([](){
             RCLCPP_INFO(rclcpp::get_logger("pairing_node"), "Advertisement Release()");
           });
    advObj_->registerProperty("Type")
           .onInterface("org.bluez.LEAdvertisement1")
           .withGetter([](){ return std::string("peripheral"); });
    advObj_->registerProperty("LocalName")
           .onInterface("org.bluez.LEAdvertisement1")
           .withGetter([](){ return std::string("pi5"); });
    advObj_->registerProperty("ServiceUUIDs")
           .onInterface("org.bluez.LEAdvertisement1")
           .withGetter([](){ return std::vector<std::string>{
             "12345678-1234-5678-1234-56789ABCDEF0"
           }; });
    advObj_->registerProperty("IncludeTxPower")
           .onInterface("org.bluez.LEAdvertisement1")
           .withGetter([](){ return true; });
    advObj_->finishRegistration();

    try {
      auto mgr = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
      mgr->callMethod("RegisterAdvertisement")
         .onInterface("org.bluez.LEAdvertisingManager1")
         .withArguments(sdbus::ObjectPath("/pairing_advertisement"),
                        std::map<std::string,sdbus::Variant>{});
      RCLCPP_INFO(get_logger(), "📡 BLE Advertising 등록 완료");
    } catch (const sdbus::Error &e) {
      RCLCPP_WARN(get_logger(), "⚠️ 광고 등록 실패: %s", e.what());
    }
  }

  // BLE 광고 해제
  void unregisterAdvertisement() {
    if (!advObj_) return;

    try {
      auto mgr = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
      mgr->callMethod("UnregisterAdvertisement")
         .onInterface("org.bluez.LEAdvertisingManager1")
         .withArguments(sdbus::ObjectPath("/pairing_advertisement"));
      advObj_.reset();
      RCLCPP_INFO(get_logger(), "🚫 BLE Advertising 해제 완료");
    } catch (const sdbus::Error &e) {
      RCLCPP_WARN(get_logger(), "⚠️ 광고 해제 실패: %s", e.what());
    }
  }

  // 연결된 디바이스 검사 및 페어링/연결
  void pollConnections() {
    if (!pairingRequested_) return;

    constexpr char CMD[] = "hcitool con | grep 'ACL' | awk '{print $3}'";
    FILE* pipe = popen(CMD, "r");
    if (!pipe) {
      RCLCPP_ERROR(get_logger(), "popen() 실패");
      return;
    }

    std::array<char,128> buf;
    while (fgets(buf.data(), buf.size(), pipe)) {
      std::string mac(buf.data());
      mac.erase(mac.find_last_not_of(" \n\r\t")+1);
      if (mac.empty() || seen_.count(mac)) continue;
      seen_.insert(mac);

      std::string path = adapterPath_ + "/dev_";
      std::string mac_path = mac;
      std::replace(mac_path.begin(), mac_path.end(), ':', '_');
      path += mac_path;

      auto dev = sdbus::createProxy(*conn_, "org.bluez", path);
      try {
        dev->callMethod("Pair").onInterface("org.bluez.Device1");
        RCLCPP_INFO(get_logger(), "Pair() 성공: %s", mac.c_str());
      } catch (const sdbus::Error &e) {
        RCLCPP_WARN(get_logger(), "Pair() 실패(%s)", e.what());
      }
      try {
        dev->callMethod("Connect").onInterface("org.bluez.Device1");
        RCLCPP_INFO(get_logger(), "Connect() 성공: %s", mac.c_str());
      } catch (const sdbus::Error &e) {
        RCLCPP_WARN(get_logger(), "Connect() 실패(%s)", e.what());
      }

      std_msgs::msg::String msg;
      msg.data = "{\"mac\":\"" + mac + "\"}";
      pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "JSON published: %s", msg.data.c_str());

      break;
    }

    pclose(pipe);
  }

  // GATT Application 등록
  void registerGattApplication() {
    gatt_app_ = std::make_unique<GattApplication>(*conn_, "/org/example/application");
    auto mgr  = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
    mgr->callMethod("RegisterApplication")
       .onInterface("org.bluez.GattManager1")
       .withArguments(sdbus::ObjectPath("/org/example/application"),
                      std::map<std::string, sdbus::Variant>{});
    RCLCPP_INFO(get_logger(), "✅ GATT Application 등록 완료");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_, advertiseTimer_;
  bool pairingRequested_{false};
  std::set<std::string> seen_;

  std::unique_ptr<sdbus::IConnection>   conn_;
  std::unique_ptr<sdbus::IObject>       advObj_;
  std::string                           adapterPath_;

  std::unique_ptr<GattApplication>      gatt_app_;

  std::unique_ptr<mqtt::async_client>   client_;
  mqtt::connect_options                 connOpts_;
  mqtt::ssl_options                     sslOpts_;

  std::string mqttHost_, mqttUser_, mqttPasswd_, sslCaPath_, proto_;
  std::string deviceId_, topicRequest_;
  int         mqttPort_{0}, keepAlive_{60};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PairingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
