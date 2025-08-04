#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <array>
#include <thread>

using json = nlohmann::json;

class ManualBtNode : public rclcpp::Node {
public:
  ManualBtNode()
  : Node("manual_bt_node"),
    socket_(io_context_,
            asio::ip::udp::endpoint(asio::ip::udp::v4(), 30080))
  {
    // 1) ROS2 퍼블리셔 생성
    pub_ = this->create_publisher<std_msgs::msg::String>("/manual_pose", 10);

    
    // 3) UDP 수신 시작
    start_receive();

    // 4) 백그라운드에서 Asio io_context 실행
    worker_ = std::thread([this](){
      io_context_.run();
    });
  }

  ~ManualBtNode() {
    io_context_.stop();
    if (worker_.joinable()) worker_.join();
  }

private:
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

  // 통신 설정
  asio::io_context io_context_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_ep_;
  std::array<char,1024> buf_;
  std::thread worker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualBtNode>());
  rclcpp::shutdown();
  return 0;
}
