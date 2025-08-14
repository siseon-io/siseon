#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <asio.hpp>
#include <array>
#include <thread>
#include <cmath>
#include <curl/curl.h>
#include <fstream>
#include <mutex>

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

      // 1. pose_xyz에 좌/우 눈 데이터가 있는지 확인합니다. (인덱스 1, 2)
      if (!j.contains("pose_xyz") || !j["pose_xyz"].is_array() || j["pose_xyz"].size() < 3) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "pose_xyz does not contain enough data for eyes. Skipping.");
          return;
      }

      const auto& lefteye_coords = j["pose_xyz"][1];
      const auto& righteye_coords = j["pose_xyz"][2];

      // 2. 좌/우 눈 데이터가 유효한지 확인합니다.
      if (!lefteye_coords.is_array() || lefteye_coords.size() != 3 || lefteye_coords[0].is_null() ||
          !righteye_coords.is_array() || righteye_coords.size() != 3 || righteye_coords[0].is_null())
      {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Left/Right eye data in pose_xyz is invalid. Skipping.");
          return;
      }

      // 3. fusion_node로 보낼 JSON을 생성합니다.
      json out_to_fusion_node;
      out_to_fusion_node["lefteye_x"] = lefteye_coords[0];
      out_to_fusion_node["lefteye_y"] = lefteye_coords[1];
      out_to_fusion_node["lefteye_z"] = lefteye_coords[2];
      out_to_fusion_node["righteye_x"] = righteye_coords[0];
      out_to_fusion_node["righteye_y"] = righteye_coords[1];
      out_to_fusion_node["righteye_z"] = righteye_coords[2];
      
      // HTTP 전송용 데이터
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_udp_data_ = j;
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
    json j;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (latest_udp_data_.is_null()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "전송할 UDP 데이터가 없습니다.");
            return;
        }
        j = latest_udp_data_;
        latest_udp_data_.clear(); // 한번 보낸 데이터는 비움
    }

    // 1. 프로필 ID 읽기
    int profile_id = 1; // 기본값
    std::ifstream profile_file("profile.json");
    if (profile_file.is_open()) {
        try {
            json profile_json;
            profile_file >> profile_json;
            if (profile_json.contains("profileId")) {
                // profileId가 문자열일 경우를 대비하여 stoi 사용
                profile_id = std::stoi(profile_json["profileId"].get<std::string>());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "profile.json 파싱 실패: %s", e.what());
        }
    }

    // 2. 새로운 JSON 양식 구성
    json payload_to_send;
    payload_to_send["profileId"] = profile_id;

    // user_coord 구성
    json user_coord;
    auto get_xyz = [](const json& arr) -> json {
        if (arr.is_array() && arr.size() == 3 && !arr[0].is_null()) {
            return {{"x", arr[0]}, {"y", arr[1]}, {"z", arr[2]}};
        }
        return {{"x", 0}, {"y", 0}, {"z", 0}}; // 유효하지 않으면 0으로 채움
    };

    if (j.contains("lepupil_xyz")) user_coord["le_pupil"] = get_xyz(j["lepupil_xyz"]);
    if (j.contains("repupil_xyz")) user_coord["re_pupil"] = get_xyz(j["repupil_xyz"]);

    const std::vector<std::string> pose_keys = {
        "nose", "le_eye", "re_eye", "le_ear", "re_ear",
        "le_shoulder", "re_shoulder", "le_elbow", "re_elbow",
        "le_wrist", "re_wrist", "le_hip", "re_hip",
        "le_knee", "re_knee", "le_ankle", "re_ankle"
    };
    
    json pose_data;
    if (j.contains("pose_xyz") && j["pose_xyz"].is_array()) {
        for (size_t i = 0; i < std::min(pose_keys.size(), j["pose_xyz"].size()); ++i) {
            pose_data[pose_keys[i]] = get_xyz(j["pose_xyz"][i]);
        }
    }
    user_coord["pose_data"] = pose_data;
    payload_to_send["userCoord"] = user_coord;

    // monitor_coord 구성
    json monitor_coord = {{"x", 0}, {"y", 0}, {"z", 0}};
    if (j.contains("pose_xyz") && j["pose_xyz"].is_array()) {
        for (const auto& point : j["pose_xyz"]) {
            if (point.is_array() && point.size() == 3 && !point[0].is_null()) {
                monitor_coord = get_xyz(point);
                break;
            }
        }
    }
    payload_to_send["monitorCoord"] = monitor_coord;

    std::string http_payload = payload_to_send.dump(4); // 4는 들여쓰기

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
  std::mutex data_mutex_;
  json latest_udp_data_;
  
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