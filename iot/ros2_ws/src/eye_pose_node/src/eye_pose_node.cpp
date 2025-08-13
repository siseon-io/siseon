#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <asio.hpp>
#include <array>
#include <thread>
#include <cmath>
#include <curl/curl.h>

using json = nlohmann::json;

class EyePosNode : public rclcpp::Node {
public:
  EyePosNode()
  : Node("eye_pose_node"),
    socket_(io_context_,
            asio::ip::udp::endpoint(asio::ip::udp::v4(), 30080))
  {

    const char* env_url = std::getenv("SERVER_URL");
    std::string default_url = (env_url && *env_url) 
                          ? std::string(env_url) 
                          : "https://i13b101.p.ssafy.io/siseon/api/raw-postures";

    // 파라미터 선언
    this->declare_parameter<std::string>("SERVER_URL", default_url);
    server_url_ = this->get_parameter("SERVER_URL").as_string();
    // libcurl 초기화
    curl_global_init(CURL_GLOBAL_DEFAULT);

    // 1) ROS2 퍼블리셔 생성
    pub_ = this->create_publisher<std_msgs::msg::String>("eye_pose", 10);

    // 3) Subscriber for the lidar_dist topic
    // person_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    //   "lidar_dist", 10,
    //   std::bind(&EyePosNode::personCallback, this, std::placeholders::_1)
    // );
    // 4) Start UDP receive
    start_receive();

    // 4) 10초마다 HTTP 전송 타이머
    http_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&EyePosNode::onHttpTimer, this)
    );

    // 5) 백그라운드에서 Asio io_context 실행
    worker_ = std::thread([this](){
      io_context_.run();
    });
  }

  ~EyePosNode() {
    io_context_.stop();
    if (worker_.joinable()) worker_.join();
    curl_global_cleanup();
  }

private:
  // Callback for lidar_dist
  // void personCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  //   latest_person_ = *msg;
  //   has_person_ = true;
  // }
  // UDP 수신 대기
  void start_receive() {
    socket_.async_receive_from(
      asio::buffer(buf_), remote_ep_,
      [this](std::error_code ec, std::size_t bytes_recvd) {
        if (!ec && bytes_recvd > 0) {
          handle_receive(bytes_recvd);
        }
        start_receive();
      });
  }
  // 수신된 UDP 페이로드 처리
  void handle_receive(std::size_t length) {
    // 1) 원본 JSON 파싱
    std::string payload(buf_.data(), length);

    // Sanitize payload by replacing "NaN" with "null"
    size_t pos = 0;
    while ((pos = payload.find("NaN", pos)) != std::string::npos) {
        payload.replace(pos, 3, "null");
    }

    try {
      auto j = json::parse(payload);

      // pose_xyz 목록에서 유효한 3D 좌표를 찾습니다.
      json valid_pose_point;
      if (j.contains("pose_xyz") && j["pose_xyz"].is_array()) {
          for (const auto& point : j["pose_xyz"]) {
              if (point.is_array() && point.size() == 3 && !point[0].is_null() && !point[1].is_null() && !point[2].is_null()) {
                  valid_pose_point = point;
                  break; // 유효한 포인트를 찾았으므로 반복 중단
              }
          }
      }

      // 유효한 좌표를 찾지 못했다면 메시지를 건너뜁니다.
      if (valid_pose_point.is_null()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "No valid pose_xyz point found. Skipping message. Payload: %s", payload.c_str());
          return;
      }

      // fusion_node로 보낼 JSON을 생성합니다.
      json out_to_fusion_node;
      out_to_fusion_node["pose_xyz"] = valid_pose_point; // 찾은 유효한 좌표만 전달

      // 다른 눈 관련 데이터도 존재하면 이름을 매핑하여 추가합니다.
      const std::map<std::string, std::string> eye_field_map = {
          {"lepupil_x", "lefteye_x"}, {"lepupil_y", "lefteye_y"}, {"lepupil_z", "lefteye_z"},
          {"repupil_x", "righteye_x"}, {"repupil_y", "righteye_y"}, {"repupil_z", "righteye_z"}
      };

      for (const auto& pair : eye_field_map) {
          if (j.contains(pair.first) && !j[pair.first].is_null()) {
              out_to_fusion_node[pair.second] = j[pair.first];
          }
      }
      
      // HTTP 전송용 데이터
      bool has_lepupil_xyz = j.contains("lepupil_xyz") && j["lepupil_xyz"].is_array() && j["lepupil_xyz"].size() == 3;
      bool has_repupil_xyz = j.contains("repupil_xyz") && j["repupil_xyz"].is_array() && j["repupil_xyz"].size() == 3;
      if (has_lepupil_xyz || has_repupil_xyz) {
        json out_to_http;
        out_to_http["profileId"] = 1; // 우선 프로필 ID는 1로 고정
        if(has_lepupil_xyz) {
            out_to_http["gaze"] = j["lepupil_xyz"];
        } else {
            out_to_http["gaze"] = j["repupil_xyz"];
        }

        if(j.contains("pose_xyz")) {
            out_to_http["pose"] = j["pose_xyz"];
        }
        batch_.push_back(std::move(out_to_http));
      }

      // Add timestamp to the JSON
      out_to_fusion_node["timestamp"] = this->now().nanoseconds();

      auto fusion_s = out_to_fusion_node.dump();
      // RCLCPP_INFO(get_logger(), "Fused JSON: %s", fusion_s.c_str());

      // Publish the message to the ROS2 topic
      std_msgs::msg::String msg;
      msg.data = fusion_s;
      pub_->publish(msg);

      
    } catch (const std::exception &e) {
      // 에러 로그는 항상 출력
      RCLCPP_ERROR(get_logger(), "JSON parse error: %s", e.what());
    }
  }

  // 10초마다 배치된 데이터를 HTTP POST
  void onHttpTimer() {
    if (batch_.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "전송할 HTTP 데이터가 없습니다.");
      return;
    }

    // 배치에서 가장 최신 데이터 하나만 사용
    json payload_json = batch_.back();
    batch_.clear();
    
    std::string http_payload = payload_json.dump();

    CURL* curl = curl_easy_init();
    if (!curl) {
      RCLCPP_ERROR(this->get_logger(), "libcurl 초기화 실패");
      return;
    }
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_URL, server_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, http_payload.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

    CURLcode res = curl_easy_perform(curl);
    if (res == CURLE_OK) {
      long code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
      RCLCPP_INFO(this->get_logger(),
                  "HTTP POST 성공: 코드=%ld, payload=%s",
                  code, http_payload.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "HTTP POST 실패: %s",
                   curl_easy_strerror(res));
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
  }
  // 멤버 변수
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr person_sub_;
  // geometry_msgs::msg::PointStamped latest_person_;
  // bool has_person_{false};
  
  // 2. 디버그 플래그 멤버 변수
  bool debug_;

  // Communication settings
  asio::io_context io_context_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_ep_;
  std::array<char,4096> buf_;
  std::thread worker_;

  rclcpp::TimerBase::SharedPtr http_timer_;
  std::string server_url_;
  std::vector<json> batch_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EyePosNode>());
  rclcpp::shutdown();
  return 0;
}