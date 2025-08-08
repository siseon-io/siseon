// src/pairing_bridge_node.cpp

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
#include <algorithm>
#include <sdbus-c++/sdbus-c++.h>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

static constexpr int QOS = 1;
using namespace std::chrono_literals;
using json = nlohmann::json;

class PairingBridgeNode
    : public rclcpp::Node
    , public virtual mqtt::callback
{
public:
    PairingBridgeNode()
    : Node("pairing_bridge_node")
    {
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug_);

        pub_ = create_publisher<std_msgs::msg::String>("/mac_addr", 10);
        conn_ = sdbus::createSystemBusConnection();
        conn_->enterEventLoopAsync();
        adapterPath_ = "/org/bluez/hci0";
        registerAgent();
        loadEnvironment();
        initMqtt();
        timer_ = create_wall_timer(1s, [this]() { pollConnections(); });
    }

    ~PairingBridgeNode() override
    {
        try { client_->disconnect()->wait(); } catch(...) {}
        unregisterAdvertisement();
        conn_->leaveEventLoop();
    }

    void connection_lost(const std::string &cause) override {
        if (debug_) RCLCPP_WARN(get_logger(), "MQTT 연결 상실: %s", cause.empty() ? "Unknown" : cause.c_str());
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        auto topic   = msg->get_topic();
        auto payload = msg->to_string();
        if (debug_) RCLCPP_INFO(get_logger(), "MQTT 수신: topic=%s payload=%s", topic.c_str(), payload.c_str());

        if (topic == topicRequest_ && !pairingRequested_) {
            try {
                json j = json::parse(payload);
                if (j.contains("profile_id")) {
                    std::string profileId = j["profile_id"];
                    if (debug_) RCLCPP_INFO(get_logger(), "📋 Profile ID 수신: %s", profileId.c_str());
                    currentProfileId_ = profileId;
                } else {
                    if (debug_) RCLCPP_WARN(get_logger(), "⚠️ JSON에 profile_id 필드가 없습니다");
                }
            } catch (const json::exception& e) {
                if (debug_) RCLCPP_WARN(get_logger(), "⚠️ JSON 파싱 실패, 일반 페어링 요청으로 처리: %s", e.what());
            }
            
            if (debug_) RCLCPP_INFO(get_logger(), "🛎 페어링 요청 수신 → BLE 광고 시작");
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

private:
    bool debug_;

    void registerAgent() {
        auto agent = sdbus::createObject(*conn_, "/pairing_agent");
        agent->registerMethod("Release").onInterface("org.bluez.Agent1").implementedAs([](){});
        agent->registerMethod("Cancel" ).onInterface("org.bluez.Agent1").implementedAs([](){});
        agent->finishRegistration();

        auto mgr = sdbus::createProxy(*conn_, "org.bluez", "/org/bluez");
        mgr->callMethod("RegisterAgent")
           .onInterface("org.bluez.AgentManager1")
           .withArguments(sdbus::ObjectPath("/pairing_agent"), std::string("NoInputNoOutput"));
        mgr->callMethod("RequestDefaultAgent")
           .onInterface("org.bluez.AgentManager1")
           .withArguments(sdbus::ObjectPath("/pairing_agent"));

        if (debug_) RCLCPP_INFO(get_logger(), "Just-Works 에이전트 등록 완료");
    }

    void makeAdapterDiscoverable() {
        auto adapter = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
        adapter->callMethod("Set").onInterface("org.freedesktop.DBus.Properties").withArguments("org.bluez.Adapter1", "Powered", sdbus::Variant(true));
        adapter->callMethod("Set").onInterface("org.freedesktop.DBus.Properties").withArguments("org.bluez.Adapter1", "Discoverable", sdbus::Variant(true));
        adapter->callMethod("Set").onInterface("org.freedesktop.DBus.Properties").withArguments("org.bluez.Adapter1", "Pairable", sdbus::Variant(true));
        if (debug_) RCLCPP_INFO(get_logger(), "🔵 어댑터 Powered/Discoverable/Pairable 설정 완료");
    }

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

    void loadEnvironment() {
        getEnv("MQTT_HOST",       mqttHost_,   true);
        getEnv("MQTT_PORT",       mqttPort_,   true);
        getEnv("DEVICE_ID",       deviceId_,   true);
        getEnv("MQTT_USER",       mqttUser_,   false);
        getEnv("MQTT_PASSWD",     mqttPasswd_, false);
        getEnv("MQTT_CA_CERT",    sslCaPath_,  false);
        getEnv("MQTT_PROTOCOL",   proto_,      false, std::string("mqtts"));
        getEnv("MQTT_KEEP_ALIVE", keepAlive_,  false, 60);
        topicRequest_ = "/request_pair/" + deviceId_;
    }

    void initMqtt() {
        std::string address = (proto_ == "mqtt" || proto_ == "tcp" ? "tcp://" : "ssl://") + mqttHost_ + ":" + std::to_string(mqttPort_);
        if (debug_) RCLCPP_INFO(get_logger(), "🔗 MQTT broker: %s", address.c_str());

        client_ = std::make_unique<mqtt::async_client>(address, "pairing_bridge_node");
        client_->set_callback(*this);

        auto sslb = mqtt::ssl_options_builder();
        if (!sslCaPath_.empty()) {
            sslb.trust_store(sslCaPath_).enable_server_cert_auth(true);
            if (debug_) RCLCPP_INFO(get_logger(), "🔒 CA cert: %s", sslCaPath_.c_str());
        } else {
            sslb.enable_server_cert_auth(false);
            if (debug_) RCLCPP_WARN(get_logger(), "⚠️ MQTT_CA_CERT 미설정: 서버 인증서 검증 OFF");
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
            if (debug_) RCLCPP_INFO(get_logger(), "🔄 MQTT 연결 시도…");
            client_->connect(connOpts_)->wait();
            if (debug_) RCLCPP_INFO(get_logger(), "✅ MQTT 연결 성공");
            client_->subscribe(topicRequest_, QOS)->wait();
            if (debug_) RCLCPP_INFO(get_logger(), "📡 구독: %s", topicRequest_.c_str());
        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(get_logger(), "❌ MQTT 연결/구독 실패: %s", e.what());
            throw;
        }
    }

    void registerAdvertisement() {
        if (advObj_) return;

        advObj_ = sdbus::createObject(*conn_, "/pairing_advertisement");
        advObj_->registerMethod("Release")
                .onInterface("org.bluez.LEAdvertisement1")
                .implementedAs([this](){ // capture this
                    if (this->debug_) RCLCPP_INFO(rclcpp::get_logger("pairing_bridge_node"), "Advertisement Release()");
                });
        advObj_->registerProperty("Type").onInterface("org.bluez.LEAdvertisement1").withGetter([](){ return std::string("peripheral"); });
        advObj_->registerProperty("LocalName").onInterface("org.bluez.LEAdvertisement1").withGetter([](){ return std::string("pi5"); });
        advObj_->registerProperty("ServiceUUIDs").onInterface("org.bluez.LEAdvertisement1").withGetter([](){ return std::vector<std::string>{"12345678-1234-5678-1234-56789ABCDEF0"}; });
        advObj_->registerProperty("IncludeTxPower").onInterface("org.bluez.LEAdvertisement1").withGetter([](){ return true; });
        advObj_->finishRegistration();

        try {
            auto mgr = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
            mgr->callMethod("RegisterAdvertisement")
               .onInterface("org.bluez.LEAdvertisingManager1")
               .withArguments(sdbus::ObjectPath("/pairing_advertisement"), std::map<std::string,sdbus::Variant>{});
            if (debug_) RCLCPP_INFO(get_logger(), "📡 BLE Advertising 등록 완료");
        } catch (const sdbus::Error &e) {
            if (debug_) RCLCPP_WARN(get_logger(), "⚠️ 광고 등록 실패: %s", e.what());
        }
    }

    void unregisterAdvertisement() {
        if (!advObj_) return;
        try {
            auto mgr = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
            mgr->callMethod("UnregisterAdvertisement")
               .onInterface("org.bluez.LEAdvertisingManager1")
               .withArguments(sdbus::ObjectPath("/pairing_advertisement"));
            advObj_.reset();
            if (debug_) RCLCPP_INFO(get_logger(), "🚫 BLE Advertising 해제 완료");
        } catch (const sdbus::Error &e) {
            if (debug_) RCLCPP_WARN(get_logger(), "⚠️ 광고 해제 실패: %s", e.what());
        }
    }

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
                if (debug_) RCLCPP_INFO(get_logger(), "Pair() 성공: %s", mac.c_str());
            } catch (const sdbus::Error &e) {
                if (debug_) RCLCPP_WARN(get_logger(), "Pair() 실패(%s)", e.what());
            }
            try {
                dev->callMethod("Connect").onInterface("org.bluez.Device1");
                if (debug_) RCLCPP_INFO(get_logger(), "Connect() 성공: %s", mac.c_str());
            } catch (const sdbus::Error &e) {
                if (debug_) RCLCPP_WARN(get_logger(), "Connect() 실패(%s)", e.what());
            }

            std_msgs::msg::String msg;
            msg.data = mac;
            pub_->publish(msg);
            if (debug_) RCLCPP_INFO(get_logger(), "MAC 주소 Published: %s", msg.data.c_str());

            break;
        }

        pclose(pipe);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_, advertiseTimer_;
    bool pairingRequested_{false};
    std::set<std::string> seen_;
    std::unique_ptr<sdbus::IConnection>   conn_;
    std::unique_ptr<sdbus::IObject>       advObj_;
    std::string                           adapterPath_;
    std::unique_ptr<mqtt::async_client>   client_;
    mqtt::connect_options                 connOpts_;
    mqtt::ssl_options                     sslOpts_;
    std::string mqttHost_, mqttUser_, mqttPasswd_, sslCaPath_, proto_;
    std::string deviceId_, topicRequest_;
    std::string currentProfileId_;
    int         mqttPort_{0}, keepAlive_{60};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PairingBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}