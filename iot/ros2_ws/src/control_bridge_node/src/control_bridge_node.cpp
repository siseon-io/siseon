#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

using json = nlohmann::json;

class ControlBridgeNode : public rclcpp::Node {
public:
    ControlBridgeNode()
        : Node("control_bridge_node"),
          mqtt_client_(make_broker_address(), "control_bridge_node") {
        
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug_);

        control_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/control_mode", 10);
        connect_mqtt();
    }

private:
    mqtt::async_client mqtt_client_;
    mqtt::connect_options conn_opts_;
    mqtt::ssl_options ssl_opts_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_pub_;
    std::string device_id_;
    std::unordered_map<std::string, std::string> device_mode_map_;
    bool debug_;

    static std::string make_broker_address() {
        const char* host = std::getenv("MQTT_HOST");
        const char* port = std::getenv("MQTT_PORT");
        const char* protocol = std::getenv("MQTT_PROTOCOL");

        if (!host || !port) {
            throw std::runtime_error("MQTT_HOST or MQTT_PORT not set");
        }

        std::string proto = "mqtts://";
        if (protocol) {
            std::string p(protocol);
            if (p == "mqtt" || p == "tcp") proto = "mqtt://";
        }
        return proto + std::string(host) + ":" + std::string(port);
    }

    void setup_ssl_options() {
        const char* ca_cert_path = std::getenv("MQTT_CA_CERT");
        if (ca_cert_path) ssl_opts_.set_trust_store(ca_cert_path);
        else ssl_opts_.set_verify(false);
    }

    void connect_mqtt() {
        conn_opts_.set_clean_session(true);

        if (make_broker_address().find("mqtts://") == 0) {
            setup_ssl_options();
            conn_opts_.set_ssl(ssl_opts_);
        }

        const char* user = std::getenv("MQTT_USER");
        const char* pass = std::getenv("MQTT_PASSWD");
        if (user && pass && strlen(user) > 0) {
            conn_opts_.set_user_name(user);
            conn_opts_.set_password(pass);
        }

        const char* id = std::getenv("DEVICE_ID");
        if (!id) throw std::runtime_error("DEVICE_ID not set");
        device_id_ = std::string(id);

        conn_opts_.set_automatic_reconnect(true);
        mqtt_client_.set_callback(cb_);

        try {
            if(debug_) RCLCPP_INFO(this->get_logger(), "MQTT 연결 중...");
            mqtt_client_.connect(conn_opts_)->wait();
            if(debug_) RCLCPP_INFO(this->get_logger(), "MQTT 연결 성공!");

            std::string control_topic = "/control_mode/" + device_id_;
            if(debug_) RCLCPP_INFO(this->get_logger(), "토픽 구독 중: %s", control_topic.c_str());
            mqtt_client_.subscribe(control_topic, 1)->wait();
            if(debug_) RCLCPP_INFO(this->get_logger(), "구독 완료!");
        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "MQTT 연결 실패: %s", e.what());
            throw;
        }
    }

    class Callback : public virtual mqtt::callback {
    public:
        Callback(ControlBridgeNode* parent) : parent_(parent) {}
        void message_arrived(mqtt::const_message_ptr msg) override {
            if (parent_->debug_) {
                RCLCPP_INFO(parent_->get_logger(), "MQTT 메시지 수신: %s", msg->get_topic().c_str());
            }
            if (msg->get_topic().find("/control_mode/") == 0) {
                parent_->handle_control_mode(msg->to_string());
            }
        }
    private:
        ControlBridgeNode* parent_;
    };

    Callback cb_{this};

    void handle_control_mode(const std::string &payload) {
        try {
            json j = json::parse(payload);
            std::string current_mode = j.value("current_mode", "");
            
            auto msg = std_msgs::msg::String();
            msg.data = current_mode;
            control_mode_pub_->publish(msg);
            if (debug_) {
                RCLCPP_INFO(this->get_logger(), "control_mode 퍼블리시: %s", current_mode.c_str());
            }

        } catch (const json::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "JSON 파싱 실패: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
