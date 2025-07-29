#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <asio.hpp>
#include <array>
#include <thread>
#include <cmath>

using json = nlohmann::json;

class EyePosNode : public rclcpp::Node {
public:
  EyePosNode()
  : Node("eye_pos_node"),
    socket_(io_context_,
            asio::ip::udp::endpoint(asio::ip::udp::v4(), 30080))
  {
    // 1) ROS2 퍼블리셔 생성
    pub_ = this->create_publisher<std_msgs::msg::String>("eye_fused_json", 10);
    // 2) person_detection 토픽 구독자 생성
    person_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "person_detection", 10,
      std::bind(&EyePosNode::personCallback, this, std::placeholders::_1)
    );
    // 3) UDP 수신 시작
    start_receive();

    // 4) 백그라운드에서 Asio io_context 실행
    worker_ = std::thread([this](){
      io_context_.run();
    });
  }

  ~EyePosNode() {
    io_context_.stop();
    if (worker_.joinable()) worker_.join();
  }

private:
  // person_detection 콜백
  void personCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    latest_person_ = *msg;
    has_person_ = true;
  }
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

  void handle_receive(std::size_t length) {
    std::string payload(buf_.data(), length);
    try {
      auto j = json::parse(payload);
      float left_x  = j.at("lepupil_x");
      float left_z  = j.at("lepupil_y");
      float right_x = j.at("repupil_x");
      float right_z = j.at("repupil_y");
      float y = 1.0f;  // 기본값
      if (has_person_) {
        y = std::abs(latest_person_.point.y)*100;
      }

      // 필드가 유효한지 간단 체크
      // (null 체크 대신 try/catch 로 예외 처리하므로 생략)

      // fused JSON 생성
      json out = {
        {"lefteye_x",  left_x},
        {"lefteye_z",  left_z},
        {"righteye_x", right_x},
        {"righteye_z", right_z},
        {"y",          y}
      };
      std::string s = out.dump();
      RCLCPP_INFO(get_logger(), "Fused JSON: %s", s.c_str());

      // ROS2 토픽으로 퍼블리시
      std_msgs::msg::String msg;
      msg.data = s;
      pub_->publish(msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "JSON parse error: %s", e.what());
    }
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EyePosNode>());
  rclcpp::shutdown();
  return 0;
}
