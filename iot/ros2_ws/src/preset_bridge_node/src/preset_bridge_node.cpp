#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <cstdlib>  // for std::getenv
#include <string>

using json = nlohmann::json;

class PresetBridgeNode : public rclcpp::Node {
public:
    PresetBridgeNode()
        : Node("preset_bridge_node"),
          mqtt_client_(make_broker_address(), "preset_bridge_node")
    {
        // ROS2 퍼블리셔 생성 - preset_pose만
        preset_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/preset_pose", 10);

        connect_mqtt();
    }

private:
    mqtt::async_client mqtt_client_;
    mqtt::connect_options conn_opts_;
    mqtt::ssl_options ssl_opts_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr preset_pub_;

    std::string device_id_;

    static std::string make_broker_address() {
        const char* host = std::getenv("MQTT_HOST");
        const char* port = std::getenv("MQTT_PORT");
        const char* protocol = std::getenv("MQTT_PROTOCOL");
        
        if (!host || !port) {
            throw std::runtime_error("MQTT_HOST or MQTT_PORT not set in .env or environment.");
        }
        
        std::string proto = "mqtts://"; // 기본값은 SSL
        if (protocol) {
            if (std::string(protocol) == "mqtt" || std::string(protocol) == "tcp") {
                proto = "mqtt://";
            } else if (std::string(protocol) == "mqtts" || std::string(protocol) == "ssl") {
                proto = "mqtts://";
            }
        }
        
        return proto + std::string(host) + ":" + std::string(port);
    }

    void setup_ssl_options() {
        // CA 인증서 파일 경로 설정
        const char* ca_cert_path = std::getenv("MQTT_CA_CERT");
        if (ca_cert_path) {
            ssl_opts_.set_trust_store(ca_cert_path);
            RCLCPP_INFO(this->get_logger(), "CA 인증서 설정: %s", ca_cert_path);
        } else {
            // CA 인증서가 없으면 서버 인증서 검증 비활성화 (개발 환경용)
            ssl_opts_.set_verify(false);
            RCLCPP_WARN(this->get_logger(), "CA 인증서 미설정 - 서버 인증서 검증 비활성화");
        }

        // 클라이언트 인증서와 키 (필요한 경우)
        const char* client_cert = std::getenv("MQTT_CLIENT_CERT");
        const char* client_key = std::getenv("MQTT_CLIENT_KEY");
        
        if (client_cert && client_key) {
            ssl_opts_.set_key_store(client_cert);
            ssl_opts_.set_private_key(client_key);
            RCLCPP_INFO(this->get_logger(), "클라이언트 인증서 설정 완료");
        }

        // SSL 버전은 OpenSSL 기본값 사용 (라이브러리에서 자동 협상)
        const char* ssl_version = std::getenv("MQTT_SSL_VERSION");
        if (ssl_version) {
            RCLCPP_INFO(this->get_logger(), "SSL 버전 설정 요청: %s (OpenSSL 자동 협상 사용)", ssl_version);
        }

        // SSL 암호화 방식 설정 (선택적)
        const char* ssl_ciphers = std::getenv("MQTT_SSL_CIPHERS");
        if (ssl_ciphers) {
            ssl_opts_.set_enabled_cipher_suites(ssl_ciphers);
            RCLCPP_INFO(this->get_logger(), "SSL 암호화 방식 설정: %s", ssl_ciphers);
        }

        // ALPN (Application-Layer Protocol Negotiation) 설정 (선택적)
        const char* alpn_protos = std::getenv("MQTT_ALPN_PROTOS");
        if (alpn_protos) {
            ssl_opts_.set_alpn_protos({alpn_protos});
            RCLCPP_INFO(this->get_logger(), "ALPN 프로토콜 설정: %s", alpn_protos);
        }
    }

    void connect_mqtt() {
        conn_opts_.set_clean_session(true);

        // 브로커 정보 로깅
        RCLCPP_INFO(this->get_logger(), "MQTT 브로커 연결 시도: %s", make_broker_address().c_str());

        // SSL 연결 여부 확인
        bool use_ssl = make_broker_address().find("mqtts://") == 0;
        if (use_ssl) {
            // SSL 옵션 설정
            setup_ssl_options();
            conn_opts_.set_ssl(ssl_opts_);
            RCLCPP_INFO(this->get_logger(), "SSL 모드로 연결 시도");
        } else {
            RCLCPP_INFO(this->get_logger(), "비 SSL 모드로 연결 시도");
        }

        // 사용자 인증 설정
        const char* user = std::getenv("MQTT_USER");
        const char* pass = std::getenv("MQTT_PASSWD");
        if (user && pass && strlen(user) > 0) {
            conn_opts_.set_user_name(user);
            conn_opts_.set_password(pass);
            RCLCPP_INFO(this->get_logger(), "사용자 인증 설정: %s", user);
        } else {
            RCLCPP_INFO(this->get_logger(), "익명 연결 (사용자 인증 없음)");
        }

        const char* id = std::getenv("DEVICE_ID");
        if (!id) {
            throw std::runtime_error("DEVICE_ID not set in .env or environment.");
        }
        device_id_ = std::string(id);
        RCLCPP_INFO(this->get_logger(), "디바이스 ID: %s", device_id_.c_str());

        // 연결 타임아웃 설정
        const char* timeout = std::getenv("MQTT_CONNECT_TIMEOUT");
        if (timeout) {
            conn_opts_.set_connect_timeout(std::chrono::seconds(std::stoi(timeout)));
            RCLCPP_INFO(this->get_logger(), "연결 타임아웃: %s초", timeout);
        } else {
            conn_opts_.set_connect_timeout(std::chrono::seconds(30)); // 기본값 30초
            RCLCPP_INFO(this->get_logger(), "연결 타임아웃: 30초 (기본값)");
        }

        // Keep Alive 설정
        const char* keep_alive = std::getenv("MQTT_KEEP_ALIVE");
        if (keep_alive) {
            conn_opts_.set_keep_alive_interval(std::stoi(keep_alive));
        } else {
            conn_opts_.set_keep_alive_interval(60); // 기본값 60초
        }

        // 자동 재연결 설정
        conn_opts_.set_automatic_reconnect(true);

        mqtt_client_.set_callback(cb_);

        try {
            RCLCPP_INFO(this->get_logger(), "MQTT 연결 중...");
            auto token = mqtt_client_.connect(conn_opts_);
            token->wait();
            
            RCLCPP_INFO(this->get_logger(), "MQTT 연결 성공!");

            // preset_coordinate 토픽만 구독
            std::string preset_topic = "/preset_coordinate/" + device_id_;
            
            RCLCPP_INFO(this->get_logger(), "토픽 구독 중: %s", preset_topic.c_str());
            mqtt_client_.subscribe(preset_topic, 1)->wait();

            RCLCPP_INFO(this->get_logger(), "구독 완료!");

        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "MQTT 연결 실패");
            RCLCPP_ERROR(this->get_logger(), "   에러 코드: %d", e.get_reason_code());
            RCLCPP_ERROR(this->get_logger(), "   에러 메시지: %s", e.what());
            
            if (use_ssl) {
                RCLCPP_ERROR(this->get_logger(), "SSL 연결 문제 해결 방법:");
                RCLCPP_ERROR(this->get_logger(), "   1. CA 인증서 파일 경로 확인: MQTT_CA_CERT");
                RCLCPP_ERROR(this->get_logger(), "   2. 브로커 주소와 포트 확인");
                RCLCPP_ERROR(this->get_logger(), "   3. 방화벽/네트워크 설정 확인");
                RCLCPP_ERROR(this->get_logger(), "   4. 테스트를 위해 mqtt:// (비SSL) 사용 고려");
            }
            
            throw;
        }
    }

    class Callback : public virtual mqtt::callback {
    public:
        Callback(PresetBridgeNode* parent) : parent_(parent) {}

        void message_arrived(mqtt::const_message_ptr msg) override {
            std::string topic = msg->get_topic();
            std::string payload = msg->to_string();

            if (topic.find("/preset_coordinate/") == 0) {
                parent_->handle_preset_coordinate(payload);
            }
        }

    private:
        PresetBridgeNode* parent_;
    };

    Callback cb_{this};

    void handle_preset_coordinate(const std::string &payload) {
        try {
            // JSON 파싱
            json j = json::parse(payload);
            
            auto msg = geometry_msgs::msg::Point();
            
            // x, y, z 값 추출 (double로)
            if (j.contains("x") && j["x"].is_number()) {
                msg.x = j["x"].get<double>();
            } else {
                RCLCPP_WARN(this->get_logger(), "'x' 값이 없거나 숫자가 아닙니다");
                msg.x = 0.0;
            }
            
            if (j.contains("y") && j["y"].is_number()) {
                msg.y = j["y"].get<double>();
            } else {
                RCLCPP_WARN(this->get_logger(), "'y' 값이 없거나 숫자가 아닙니다");
                msg.y = 0.0;
            }
            
            if (j.contains("z") && j["z"].is_number()) {
                msg.z = j["z"].get<double>();
            } else {
                RCLCPP_WARN(this->get_logger(), "'z' 값이 없거나 숫자가 아닙니다");
                msg.z = 0.0;
            }
            
            preset_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "preset_pose 퍼블리시: x=%.3f, y=%.3f, z=%.3f", 
                       msg.x, msg.y, msg.z);
                       
        } catch (const json::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "JSON 파싱 실패: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "   받은 payload: %s", payload.c_str());
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "preset coordinate 처리 실패: %s", e.what());
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