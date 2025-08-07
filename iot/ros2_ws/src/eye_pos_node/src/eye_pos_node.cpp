#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <asio.hpp>
#include <array>
#include <thread>
#include <cmath>
#include <curl/curl.h>

using json = nlohmann::json;

class EyePosNode : public rclcpp::Node {
public:
  EyePosNode()
  : Node("eye_pos_node"),
    socket_(io_context_,
            asio::ip::udp::endpoint(asio::ip::udp::v4(), 30080))
  {

    const char* env_url = std::getenv("SERVER_URL");
    std::string default_url = (env_url && *env_url) 
                          ? std::string(env_url) 
                          : "http://i13b101.p.ssafy.io:8080/api/raw-postures";

    // 파라미터 선언
    this->declare_parameter<std::string>("SERVER_URL", default_url);
    server_url_ = this->get_parameter("SERVER_URL").as_string();
    // libcurl 초기화
    curl_global_init(CURL_GLOBAL_DEFAULT);

    // 1) ROS2 퍼블리셔 생성
    pub_ = this->create_publisher<std_msgs::msg::String>("eye_pose", 10);

    // 2) lidar_dist 토픽 구독자 생성
    person_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "lidar_dist", 10,
      std::bind(&EyePosNode::personCallback, this, std::placeholders::_1)
    );
    // 3) UDP 수신 시작
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
  // lidar_dist 콜백
  void personCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    latest_person_ = *msg;
    has_person_ = true;
  }
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
    try {
      auto j = json::parse(payload);
      float left_x  = j.at("lepupil_x");
      float left_z  = j.at("lepupil_y");
      float right_x = j.at("repupil_x");
      float right_z = j.at("repupil_y");
      auto pose_xy = j.at("pose_xy");
      float y = 1.0f;  // 기본값
      if (has_person_) {
        y = std::abs(latest_person_.point.y)*100;
      }

      
      // json out_to_http = {
      //   {"lepupil_x",    left_x},
      //   {"lepupil_y",    left_z},
      //   {"repupil_x",    right_x},
      //   {"repupil_y",    right_z},
      //   {"z", y},
      //   {"pose_xy",      pose_xy}, 
      // };
      // http test
      json out_to_http = {
        {"profileId", 1},
        {"x" , left_x},
        {"y", left_z},
        {"z", y}
      };
      batch_.push_back(std::move(out_to_http));

      // out_to_fusion_node JSON 생성
      json out_to_fusion_node = {
        {"lefteye_x",  left_x},
        {"lefteye_z",  left_z},
        {"righteye_x", right_x},
        {"righteye_z", right_z},
        {"y",          y}
      };
      auto fusion_s = out_to_fusion_node.dump();
      RCLCPP_INFO(get_logger(), "Fused JSON: %s", fusion_s.c_str());

      // ROS2 토픽으로 퍼블리시
      std_msgs::msg::String msg;
      msg.data = fusion_s;
      pub_->publish(msg);

      
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "JSON parse error: %s", e.what());
    }
  }

  // 10초마다 배치된 데이터를 HTTP POST
  void onHttpTimer() {
    if (batch_.empty()) {
      RCLCPP_INFO(this->get_logger(), "전송할 배치 데이터가 없습니다.");
      return;
    }
    json arr = batch_;
    std::string http_payload = arr[0].dump();

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
                  "HTTP POST 성공: 코드=%ld, 배치 크기=%zu",
                  code, batch_.size());
      batch_.clear();
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
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr person_sub_;  // 추가
  geometry_msgs::msg::PointStamped latest_person_;
  bool has_person_{false};

  // 통신 설정
  asio::io_context io_context_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_ep_;
  std::array<char,1024> buf_;
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
