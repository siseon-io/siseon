#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <string>
#include <vector>

using json = nlohmann::json;

class PresetBridgeNode : public rclcpp::Node {
public:
    PresetBridgeNode()
        : Node("preset_bridge_node"),
          mqtt_client_(make_broker_address(), "preset_bridge_node")
    {
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug_);

        const char* id = std::getenv("DEVICE_ID");
        if (!id) {
            throw std::runtime_error("DEVICE_ID not set");
        }
        device_id_ = std::string(id);

        preset_pub_ = this->create_publisher<std_msgs::msg::String>("/preset_pose", 10);
        connect_mqtt();
    }

private:
    mqtt::async_client mqtt_client_;
    mqtt::connect_options conn_opts_;
    mqtt::ssl_options ssl_opts_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr preset_pub_;
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
            parent_->handle_coordinate_data(msg->to_string());
        }

    private:
        PresetBridgeNode* parent_;
    };

    Callback cb_{this};

    void handle_coordinate_data(const std::string& json_payload)
    {
        try {
            auto j = json::parse(json_payload);
            
            double lefteye_x = j.value("lefteyeX", 0.0);
            double lefteye_y = j.value("lefteyeY", 0.0);
            double lefteye_z = j.value("lefteyeZ", 0.0);
            double righteye_x = j.value("righteyeX", 0.0);
            double righteye_y = j.value("righteyeY", 0.0);
            double righteye_z = j.value("righteyeZ", 0.0);

            // JSON 문자열로 다시 생성
            json output_json;
            output_json["lefteye_x"] = lefteye_x;
            output_json["lefteye_y"] = lefteye_y;
            output_json["lefteye_z"] = lefteye_z;
            output_json["righteye_x"] = righteye_x;
            output_json["righteye_y"] = righteye_y;
            output_json["righteye_z"] = righteye_z;

            std_msgs::msg::String msg;
            msg.data = output_json.dump();

            preset_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "preset_pose published: lefteyeX=%.3f, lefteyeY=%.3f, lefteyeZ=%.3f, righteyeX=%.3f, righteyeY=%.3f, righteyeZ=%.3f", 
                       lefteye_x, lefteye_y, lefteye_z, righteye_x, righteye_y, righteye_z);
        } catch (const json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s. Payload: %s", e.what(), json_payload.c_str());
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
