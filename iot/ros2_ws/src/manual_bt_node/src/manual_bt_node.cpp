// src/manual_bt_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include <sdbus-c++/sdbus-c++.h>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring> // for std::memcpy
#include <sstream>
#include <iomanip> // for std::setw, std::setfill

using namespace std::chrono_literals;

// xml2cpp로 생성된 GATT 서비스·특성 어댑터 헤더
#include "gatt_service_adaptor.h"
#include "gatt_characteristic_adaptor.h"

// 전방 선언
class ManualBTNode;
class GattApplication;



// ───────────────────────────────── 유틸 (LE 파서)
static inline uint32_t read_le32(const std::vector<uint8_t>& d, size_t off) {
    return static_cast<uint32_t>(d[off]) |
           (static_cast<uint32_t>(d[off+1]) << 8) |
           (static_cast<uint32_t>(d[off+2]) << 16) |
           (static_cast<uint32_t>(d[off+3]) << 24);
  }
  static inline uint16_t read_le16(const std::vector<uint8_t>& d, size_t off) {
    return static_cast<uint16_t>(d[off]) |
           (static_cast<uint16_t>(d[off+1]) << 8);
  }

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

    // 고유한 서비스 UUID
    std::string UUID() override { return "12345678-1234-5678-1234-56789ABCDEF0"; }
    bool Primary() override { return true; }
};

// ── GattCharacteristic 어댑터 ─────────────────────────────────────────
class GattCharacteristic : public sdbus::AdaptorInterfaces<org::bluez::GattCharacteristic1_adaptor>
{
public:
    GattCharacteristic(sdbus::IConnection& connection,
                       std::string objectPath,
                       ManualBTNode* node) // ManualBTNode 포인터를 받도록 수정
    : AdaptorInterfaces(connection, std::move(objectPath))
    , node_(node) // node_ 멤버 변수 초기화
    {
        registerAdaptor();
    }
    ~GattCharacteristic() { unregisterAdaptor(); }

    // Read/Write 메서드 선언
    std::vector<uint8_t> ReadValue(const std::map<std::string, sdbus::Variant>& options) override;
    void WriteValue(const std::vector<uint8_t>& value,
                    const std::map<std::string, sdbus::Variant>& options) override;

    // 고유한 특성 UUID
    std::string UUID() override { return "12345678-1234-5678-1234-56789ABCDEF1"; }
    sdbus::ObjectPath Service() override {
        return sdbus::ObjectPath("/org/example/application/service0");
    }
    std::vector<std::string> Flags() override { return {"read", "write","write-without-response"}; }

private:
    ManualBTNode* node_; // ROS 2 노드에 접근하기 위한 포인터
    std::vector<uint8_t> characteristic_value_{ 'H','e','l','l','o' }; // 초기값
};

// ── GattApplication: ObjectManager 인터페이스를 편의 API로 등록 ─────────
class GattApplication
{
public:
    GattApplication(sdbus::IConnection& connection, const std::string& objectPath, ManualBTNode* node)
    : conn_(connection)
    , objPath_(objectPath)
    {
        // ObjectManager 전체를 등록
        obj_ = sdbus::createObject(conn_, objPath_);
        obj_->addObjectManager();
        obj_->finishRegistration();

        // GATT 서비스/특성 어댑터 생성
        service_ = std::make_unique<GattService>(conn_, objPath_ + "/service0");
        // GattCharacteristic 생성 시 ManualBTNode 포인터를 전달
        characteristic_ = std::make_unique<GattCharacteristic>(conn_, objPath_ + "/service0/char0", node);
    }

    ~GattApplication() = default;

    // BlueZ가 관리 객체 정보를 요청할 때 호출하는 메서드
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
            props["UUID"] = sdbus::Variant(service_->UUID());
            props["Primary"] = sdbus::Variant(service_->Primary());
            response[sdbus::ObjectPath(service_->getObjectPath())]
                    ["org.bluez.GattService1"] = props;
        }

        // Characteristic 등록 정보
        {
            std::map<std::string, sdbus::Variant> props;
            props["UUID"] = sdbus::Variant(characteristic_->UUID());
            props["Service"] = sdbus::Variant(sdbus::ObjectPath(characteristic_->Service()));
            props["Flags"] = sdbus::Variant(characteristic_->Flags());
            response[sdbus::ObjectPath(characteristic_->getObjectPath())]
                    ["org.bluez.GattCharacteristic1"] = props;
        }

        return response;
    }

private:
    sdbus::IConnection& conn_;
    std::string objPath_;
    std::unique_ptr<sdbus::IObject> obj_;
    std::unique_ptr<GattService> service_;
    std::unique_ptr<GattCharacteristic> characteristic_;
};


// ── ROS 2 노드 클래스 ───────────────────────────────────────────────────
class ManualBTNode : public rclcpp::Node
{
public:
    ManualBTNode()
    : Node("manual_bt_node")
    {
        // 1. /mac_addr 토픽 구독자 생성
        mac_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mac_addr", 10,
            std::bind(&ManualBTNode::macCallback, this, std::placeholders::_1));

        // 2. /manual_pose 토픽 퍼블리셔 생성 (String으로 변경)
        pose_pub_ = this->create_publisher<std_msgs::msg::String>("/manual_pose", 10);

        // 3. D-Bus 연결 및 GATT 애플리케이션 등록
        try {
            conn_ = sdbus::createSystemBusConnection();
            conn_->enterEventLoopAsync();
            adapterPath_ = "/org/bluez/hci0";
            // registerAgent(); // 필요 시 주석 해제
            registerGattApplication();
        } catch (const sdbus::Error& e) {
            RCLCPP_ERROR(this->get_logger(), "D-Bus 연결 실패: %s", e.what());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ Manual BT Node (GATT 서버) 시작됨. /mac_addr 토픽을 기다립니다...");
    }

    ~ManualBTNode()
    {
        // GATT 애플리케이션 해제
        if (gatt_app_ && conn_) {
            try {
                auto mgr = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
                mgr->callMethod("UnregisterApplication")
                   .onInterface("org.bluez.GattManager1")
                   .withArguments(sdbus::ObjectPath("/org/example/application"));
                RCLCPP_INFO(get_logger(), "🚫 GATT Application 해제 완료");
            } catch (const sdbus::Error& e) {
                RCLCPP_ERROR(get_logger(), "GATT Application 해제 실패: %s", e.what());
            }
        }

        if (conn_) {
            conn_->leaveEventLoop();
        }
    }

    // GattCharacteristic이 호출할 데이터 발행 함수
    void publishPose(const std::vector<uint8_t>& data) {
        // 데이터 포맷: x(1바이트), y(1바이트), z(1바이트) = 총 3바이트
        if (data.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "수신 데이터가 너무 짧습니다 (3바이트 필요). Size: %zu", data.size());
            return;
        }
        // 데이터를 받은 이후부터 찍을 timestamp
        // const uint64_t time = static_cast<uint64_t>(this->now().nanoseconds());

        // 바이트 배열에서 값을 추출
        // float x = static_cast<float>(static_cast<int8_t>(data[0]));
        // float y = static_cast<float>(static_cast<int8_t>(data[1]));
        // float z = static_cast<float>(static_cast<int8_t>(data[2]));
        const uint8_t  ver   = data[0];
        const uint32_t epoch = read_le32(data, 1);
        const uint16_t msec  = read_le16(data, 5);

        const uint64_t t0_ns =
            (uint64_t)epoch * 1000000000ULL +
            (uint64_t)msec  * 1000000ULL;

        const uint64_t rx_ns = (uint64_t)this->now().nanoseconds();
        const uint64_t latency_ns = (rx_ns > t0_ns) ? (rx_ns - t0_ns) : 0ULL;

        // --- 오프셋 제거: 첫 패킷 기준 고정 ---
        static bool     have_offset = false;
        static int64_t  offset_ns   = 0;
        if (!have_offset) {
            offset_ns = (int64_t)latency_ns;
            have_offset = true;
        }
        const int64_t net_ns = (int64_t)latency_ns - offset_ns;  // ≈ 실질 지터/지연

        const float x = (float)(int8_t)data[7];
        const float y = (float)(int8_t)data[8];
        const float z = (float)(int8_t)data[9];

        // 로그: ms (오프셋 제거한 값과 원래 측정치 둘 다)
        RCLCPP_INFO(this->get_logger(),
            "[BLE] Δabs=%.3f ms, Δnet=%.3f ms | xyz=(%.0f,%.0f,%.0f) v=0x%02X",
            (double)latency_ns/1e6, (double)net_ns/1e6, x, y, z, ver);
        nlohmann::json json_data;
        json_data["x"] = x;
        json_data["y"] = y;
        json_data["z"] = z;
        json_data["time"] = latency_ns;

        std_msgs::msg::String string_msg;
        string_msg.data = json_data.dump();

        // /manual_pose 토픽으로 발행
        pose_pub_->publish(string_msg);

        // RCLCPP_INFO(this->get_logger(), "GATT 데이터 수신 및 발행: [x: %.3f, y: %.3f, z: %.3f]", x, y, z);
    }

private:
    // /mac_addr 토픽 콜백 함수
    void macCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string new_mac = msg->data;
        if (target_mac_ != new_mac) {
            RCLCPP_INFO(this->get_logger(), "새로운 타겟 MAC 주소 수신: %s", new_mac.c_str());
            target_mac_ = new_mac;
        }
    }

    // GATT 애플리케이션을 BlueZ에 등록하는 함수
    void registerGattApplication() {
        // GattApplication 생성 시 'this' 포인터를 전달하여 ROS 노드와 연결
        gatt_app_ = std::make_unique<GattApplication>(*conn_, "/org/example/application", this);
        
        try {
            auto mgr = sdbus::createProxy(*conn_, "org.bluez", adapterPath_);
            mgr->callMethod("RegisterApplication")
               .onInterface("org.bluez.GattManager1")
               .withArguments(sdbus::ObjectPath("/org/example/application"),
                              std::map<std::string, sdbus::Variant>{});
            RCLCPP_INFO(get_logger(), "✅ GATT Application 등록 완료");
        } catch (const sdbus::Error& e) {
            RCLCPP_ERROR(get_logger(), "GATT Application 등록 실패: %s", e.what());
        }
    }

    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mac_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_pub_;

    std::string target_mac_;

    std::unique_ptr<sdbus::IConnection> conn_;
    std::string adapterPath_;
    std::unique_ptr<GattApplication> gatt_app_;
};


// ── GattCharacteristic 메서드 구현 ───────────────────────────────────

// ReadValue 구현 (호출 시 현재 값을 반환)
std::vector<uint8_t> GattCharacteristic::ReadValue(
  const std::map<std::string, sdbus::Variant>&)
{
    // 바이트 배열을 16진수 문자열로 변환하여 로그 출력
    std::ostringstream oss;
    for (auto b : characteristic_value_) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << ' ';
    }
    RCLCPP_INFO(rclcpp::get_logger("gatt_server"),
                "ReadValue 호출, 반환 데이터: [%s]",
                oss.str().c_str());

    return characteristic_value_;
}

// WriteValue 구현 (데이터 수신 시 호출됨)
void GattCharacteristic::WriteValue(
  const std::vector<uint8_t>& value,
  const std::map<std::string, sdbus::Variant>&)
{
    // 수신된 데이터를 16진수 문자열로 변환하여 로그 출력
    std::ostringstream oss;
    for (auto b : value) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << ' ';
    }
    // RCLCPP_INFO(rclcpp::get_logger("gatt_server"),
    //             "WriteValue 호출, 수신 데이터: [%s]",
    //             oss.str().c_str());

    // 내부 값 갱신
    characteristic_value_ = value;

    // ROS 2 노드의 publishPose 함수를 호출하여 토픽 발행
    if (node_) {
        node_->publishPose(value);
    }
}


// ── main 함수 ────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualBTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
