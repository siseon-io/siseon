#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sdbus-c++/sdbus-c++.h>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring> // for strcasecmp

using namespace std::chrono_literals;

class ManualBTNode : public rclcpp::Node
{
public:
    ManualBTNode()
    : Node("manual_bt_node")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>(
            "serviceUuid ",
            "12345678-1234-5678-1234-56789ABCDEF0");
        this->declare_parameter<std::string>(
            "characteristicUuid",
            "12345678-1234-5678-1234-56789ABCDEF1");
        this->declare_parameter<int>(
            "poll_interval_ms", 50);

        // 구독/퍼블리셔
        mac_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mac_addr", 10,
            std::bind(&ManualBTNode::macCallback,
                        this, std::placeholders::_1));
        pose_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/manual_pose", 10);

        // D-Bus 시스템 버스 연결
        conn_ = sdbus::createSystemBusConnection();

        // ObjectManager 프록시 생성 및 시그널 리스너
        obj_mgr_ = sdbus::createProxy(*conn_, "org.bluez", "/");

        // InterfacesAdded 시그널 핸들러
        obj_mgr_->uponSignal("InterfacesAdded")
            .onInterface("org.freedesktop.DBus.ObjectManager")
            .call([this](const sdbus::ObjectPath& path, const std::map<std::string, std::map<std::string, sdbus::Variant>>& interfaces) {
                (void)path;
                (void)interfaces;
            });
        obj_mgr_->finishRegistration();

        // 주기 타이머
        auto interval = std::chrono::milliseconds(
            this->get_parameter("poll_interval_ms").as_int());
        timer_ = this->create_wall_timer(
            interval,
            std::bind(&ManualBTNode::poll, this));
    }

private:
    // /mac_addr 콜백
    void macCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "새로운 MAC 주소 수신: %s", msg->data.c_str());
        if (target_mac_ != msg->data) {
            target_mac_ = msg->data;
            char_path_.clear();
        }
    }

    // 주기 실행 함수
    void poll()
    {
        if (target_mac_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "대기 중: /mac_addr 메시지 수신 전");
            return;
        }

        std::string dev_path = "/org/bluez/hci0/dev_" + macToPath(target_mac_);

        try {
            auto dev_proxy = sdbus::createProxy(*conn_, "org.bluez", dev_path);

            bool connected = dev_proxy->getProperty("Connected")
                                 .onInterface("org.bluez.Device1")
                                 .get<bool>();

            if (!connected) {
                RCLCPP_DEBUG(this->get_logger(), "장치 연결 안 됨: %s", target_mac_.c_str());
                return;
            }

            if (char_path_.empty()) {
                if (!discoverCharacteristic(dev_path)) {
                    RCLCPP_WARN(this->get_logger(), "지정된 UUID의 Characteristic을 찾을 수 없음");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "발견된 Characteristic 경로: %s", char_path_.c_str());
            }

            auto char_proxy = sdbus::createProxy(*conn_, "org.bluez", char_path_);
            std::map<std::string, sdbus::Variant> opts;
            
            // ########## API 수정 ##########
            // '=' 대입 초기화 대신 '()' 직접 초기화 구문을 사용합니다.
            // sdbus-c++에서는 메서드 호출 결과를 받을 때 대입(=)을 사용해야 합니다.
            std::vector<uint8_t> raw_value = char_proxy->callMethod("ReadValue")
                                                 .onInterface("org.bluez.GattCharacteristic1")
                                                 .withArguments(opts);

            std::string json_str(raw_value.begin(), raw_value.end());

            auto msg = std_msgs::msg::String();
            msg.data = json_str;
            pose_pub_->publish(msg);

        } catch (const sdbus::Error& e) {
            RCLCPP_WARN(this->get_logger(), "D-Bus 오류 (%s): %s", target_mac_.c_str(), e.what());
            char_path_.clear();
        }
    }

    // GetManagedObjects를 이용해 Characteristic 찾기
    bool discoverCharacteristic(const std::string& dev_path)
    {
        using ObjectMap = std::map<sdbus::ObjectPath, std::map<std::string, std::map<std::string, sdbus::Variant>>>;

        // ########## API 수정 ##########
        // '=' 대입 초기화 대신 '()' 직접 초기화 구문을 사용합니다.
        ObjectMap objects = obj_mgr_->callMethod("GetManagedObjects")
                                .onInterface("org.freedesktop.DBus.ObjectManager");

        std::string target_uuid = this->get_parameter("characteristicUuid").as_string();

        for (const auto& [path, ifs] : objects) {
            if (path.rfind(dev_path, 0) != 0) continue;

            auto it = ifs.find("org.bluez.GattCharacteristic1");
            if (it == ifs.end()) continue;

            const auto& props = it->second;
            auto uuid_it = props.find("UUID");
            if (uuid_it != props.end()) {
                std::string uuid = uuid_it->second.get<std::string>();
                if (strcasecmp(uuid.c_str(), target_uuid.c_str()) == 0) {
                    char_path_ = path;
                    return true;
                }
            }
        }
        return false;
    }

    // "AA:BB:CC:DD:EE:FF" -> "AA_BB_CC_DD_EE_FF"
    static std::string macToPath(std::string mac)
    {
        std::replace(mac.begin(), mac.end(), ':', '_');
        return mac;
    }

    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mac_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string target_mac_, char_path_;

    std::unique_ptr<sdbus::IConnection> conn_;
    std::unique_ptr<sdbus::IProxy>      obj_mgr_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualBTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
