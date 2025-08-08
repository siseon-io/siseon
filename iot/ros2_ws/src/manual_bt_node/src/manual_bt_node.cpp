// src/manual_bt_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>

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
    std::vector<std::string> Flags() override { return {"read", "write"}; }

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

        // 2. /manual_pose 토픽 퍼블리셔 생성
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/manual_pose", 10);

        // 3. D-Bus 연결 및 GATT 애플리케이션 등록
        try {
            conn_ = sdbus::createSystemBusConnection();
            conn_->enterEventLoopAsync();
            adapterPath_ = "/org/bluez/hci0";
            // registerAgent(); // 필요 시 주석 해제
            registerGattApplication();
        } catch (const sdbus::Error& e) {
            // RCLCPP_ERROR(this->get_logger(), "D-Bus 연결 실패: %s", e.what());
            return;
        }
        
        // RCLCPP_INFO(this->get_logger(), "✅ Manual BT Node (GATT 서버) 시작됨. /mac_addr 토픽을 기다립니다...");
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
        // 데이터 포맷: x(float, 4바이트), y(float, 4바이트), z(float, 4바이트) = 총 12바이트
        if (data.size() < 3) {
            // RCLCPP_WARN(this->get_logger(), "수신 데이터가 너무 짧습니다 (12바이트 필요). Size: %zu", data.size());
            return;
        }

        geometry_msgs::msg::Point point_msg;
        
        // 바이트 배열에서 float 값을 추출
        float x = static_cast<float>(static_cast<int8_t>(data[0]));
        float y = static_cast<float>(static_cast<int8_t>(data[1]));
        float z = static_cast<float>(static_cast<int8_t>(data[2]));
        point_msg.x = x; point_msg.y = y; point_msg.z = z;
        // /manual_pose 토픽으로 발행
        pose_pub_->publish(point_msg);

        // RCLCPP_INFO(this->get_logger(), "GATT 데이터 수신 및 발행: [x: %.3f, y: %.3f, z: %.3f]",
        //             point_msg.x, point_msg.y, point_msg.z);
    }

private:
    // /mac_addr 토픽 콜백 함수
    void macCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string new_mac = msg->data;
        if (target_mac_ != new_mac) {
            // RCLCPP_INFO(this->get_logger(), "새로운 타겟 MAC 주소 수신: %s", new_mac.c_str());
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
            // RCLCPP_INFO(get_logger(), "✅ GATT Application 등록 완료");
        } catch (const sdbus::Error& e) {
            // RCLCPP_ERROR(get_logger(), "GATT Application 등록 실패: %s", e.what());
        }
    }

    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mac_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub_;

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
    // RCLCPP_INFO(rclcpp::get_logger("gatt_server"),
    //             "ReadValue 호출, 반환 데이터: [%s]",
    //             oss.str().c_str());

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
