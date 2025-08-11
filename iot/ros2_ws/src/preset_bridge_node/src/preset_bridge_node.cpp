#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <string>

using json = nlohmann::json;

class PresetBridgeNode : public rclcpp::Node {
public:
    PresetBridgeNode()
        : Node("preset_bridge_node"),
          mqtt_client_(make_broker_address(), "preset_bridge_node")
    {
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug_);

        preset_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/preset_pose", 10);
        connect_mqtt();
    }

private:
    mqtt::async_client mqtt_client_;
    mqtt::connect_options conn_opts_;
    mqtt::ssl_options ssl_opts_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr preset_pub_;
    bool debug_;
    std::string device_id_;

    static std::string make_broker_address() {
        const char* host = std::getenv("MQTT_HOST");
        const char* port = std::getenv("MQTT_PORT");
        const char* protocol = std::getenv("MQTT_PROTOCOL");
        if (!host || !port) {
            throw std::runtime_error("MQTT_HOST or MQTT_PORT not set");
        }
        std::string proto = (protocol && std::string(protocol) == "mqtt") ? "mqtt://" : "mqtts://";
        return proto + std::string(host) + ":" + std::string(port);
    }

    void setup_ssl_options() {
        const char* ca_cert_path = std::getenv("MQTT_CA_CERT");
        if (ca_cert_path) {
            ssl_opts_.set_trust_store(ca_cert_path);
        } else {
            ssl_opts_.set_verify(false);
        }
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
        if (!id) {
            throw std::runtime_error("DEVICE_ID not set");
        }
        device_id_ = std::string(id);

        conn_opts_.set_automatic_reconnect(true);
        mqtt_client_.set_callback(cb_);

        try {
            if(debug_) RCLCPP_INFO(this->get_logger(), "MQTT 연결 중...");
            mqtt_client_.connect(conn_opts_)->wait();
            if(debug_) RCLCPP_INFO(this->get_logger(), "MQTT 연결 성공!");

            std::string preset_topic = "/preset_coordinate/" + device_id_;
            if(debug_) RCLCPP_INFO(this->get_logger(), "토픽 구독 중: %s", preset_topic.c_str());
            mqtt_client_.subscribe(preset_topic, 1)->wait();
            if(debug_) RCLCPP_INFO(this->get_logger(), "구독 완료!");

        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "MQTT 연결 실패: %s", e.what());
            throw;
        }
    }

    class Callback : public virtual mqtt::callback {
    public:
        Callback(PresetBridgeNode* parent) : parent_(parent) {}

        void message_arrived(mqtt::const_message_ptr msg) override {
            if (parent_->debug_) {
                RCLCPP_INFO(parent_->get_logger(), "MQTT 메시지 수신: %s", msg->get_topic().c_str());
            }
            parent_->handle_preset_coordinate(msg->to_string());
        }

    private:
        PresetBridgeNode* parent_;
    };

    Callback cb_{this};

    void handle_preset_coordinate(const std::string &payload) {
        try {
            json j = json::parse(payload);
            auto msg = geometry_msgs::msg::Point();
            msg.x = j.value("x", 0.0);
            msg.y = j.value("y", 0.0);
            msg.z = j.value("z", 0.0);
            
            preset_pub_->publish(msg);
            if (debug_) {
                RCLCPP_INFO(this->get_logger(), "preset_pose 퍼블리시: x=%.3f, y=%.3f, z=%.3f", msg.x, msg.y, msg.z);
            }
        } catch (const json::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "JSON 파싱 실패: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PresetBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
