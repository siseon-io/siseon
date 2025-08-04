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
          mqtt_client_(make_broker_address(), "control_bridge_node")
    {
        control_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/control_mode", 10);
        connect_mqtt();
        device_mode_map_[device_id_] = "auto"; 
    }

private:
    mqtt::async_client mqtt_client_;
    mqtt::connect_options conn_opts_;
    mqtt::ssl_options ssl_opts_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_pub_;
    std::string device_id_;
    std::unordered_map<std::string, std::string> device_mode_map_;

    static std::string make_broker_address() {
        const char* host = std::getenv("MQTT_HOST");
        const char* port = std::getenv("MQTT_PORT");
        const char* protocol = std::getenv("MQTT_PROTOCOL");

        if (!host || !port) {
            throw std::runtime_error("MQTT_HOST or MQTT_PORT not set in .env or environment.");
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

        const char* client_cert = std::getenv("MQTT_CLIENT_CERT");
        const char* client_key = std::getenv("MQTT_CLIENT_KEY");
        if (client_cert && client_key) {
            ssl_opts_.set_key_store(client_cert);
            ssl_opts_.set_private_key(client_key);
        }

        const char* ssl_ciphers = std::getenv("MQTT_SSL_CIPHERS");
        if (ssl_ciphers) ssl_opts_.set_enabled_cipher_suites(ssl_ciphers);

        const char* alpn_protos = std::getenv("MQTT_ALPN_PROTOS");
        if (alpn_protos) ssl_opts_.set_alpn_protos({alpn_protos});
    }

    void connect_mqtt() {
        conn_opts_.set_clean_session(true);

        bool use_ssl = make_broker_address().find("mqtts://") == 0;
        if (use_ssl) {
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
        if (!id) throw std::runtime_error("DEVICE_ID not set in .env or environment.");
        device_id_ = std::string(id);

        const char* timeout = std::getenv("MQTT_CONNECT_TIMEOUT");
        conn_opts_.set_connect_timeout(std::chrono::seconds(timeout ? std::stoi(timeout) : 30));

        const char* keep_alive = std::getenv("MQTT_KEEP_ALIVE");
        conn_opts_.set_keep_alive_interval(keep_alive ? std::stoi(keep_alive) : 60);

        conn_opts_.set_automatic_reconnect(true);
        mqtt_client_.set_callback(cb_);

        try {
            auto token = mqtt_client_.connect(conn_opts_);
            token->wait();

            std::string control_topic = "/control_mode/" + device_id_;
            mqtt_client_.subscribe(control_topic, 1)->wait();

            // optional: 초기 상태 설정
            device_mode_map_[device_id_] = "preset";

        } catch (const mqtt::exception &e) {
            throw;
        }
    }

    class Callback : public virtual mqtt::callback {
    public:
        Callback(ControlBridgeNode* parent) : parent_(parent) {}

        void message_arrived(mqtt::const_message_ptr msg) override {
            std::string topic = msg->get_topic();
            std::string payload = msg->to_string();
            if (topic.find("/control_mode/") == 0) {
                parent_->handle_control_mode(payload);
            }
        }
    private:
        ControlBridgeNode* parent_;
    };

    Callback cb_{this};

    void handle_control_mode(const std::string &payload) {
        try {
            json j = json::parse(payload);
            if (!j.contains("profile_id") || !j.contains("previous_mode") || !j.contains("current_mode")) {
                RCLCPP_ERROR(this->get_logger(), "필수 필드 누락");
                return;
            }

            std::string current_mode = j["current_mode"].get<std::string>();
            std::string previous_mode = j["previous_mode"].get<std::string>();
            std::string profile_id = j["profile_id"].get<std::string>();
            std::string device_id = device_id_;

            std::string current_stored_mode = device_mode_map_.count(device_id) ? device_mode_map_[device_id] : "unknown";
            if (current_stored_mode != previous_mode) {
                RCLCPP_WARN(this->get_logger(), "전이 거부됨 - 이전 모드 불일치: 저장된=%s, 요청된=%s",
                            current_stored_mode.c_str(), previous_mode.c_str());
                return;
            }

            std::vector<std::string> valid_modes = {"preset", "fix", "manual", "auto", "stop"};
            if (std::find(valid_modes.begin(), valid_modes.end(), current_mode) == valid_modes.end()) {
                RCLCPP_WARN(this->get_logger(), "알 수 없는 control_mode: %s", current_mode.c_str());
            }

            device_mode_map_[device_id] = current_mode;

            auto msg = std_msgs::msg::String();
            msg.data = current_mode;
            control_mode_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "control_mode 전이 완료: %s → %s",
                        previous_mode.c_str(), current_mode.c_str());

        } catch (const json::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "JSON 파싱 실패: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "예외 발생: %s", e.what());
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
