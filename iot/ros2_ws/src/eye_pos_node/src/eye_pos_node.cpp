#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <asio.hpp>               // standalone Asio
#include <array>
#include <thread>

using json = nlohmann::json;

class EyePosNode : public rclcpp::Node {
public:
  EyePosNode()
  : Node("eye_pos_node"),
    socket_(io_context_,
            asio::ip::udp::endpoint(asio::ip::udp::v4(), 30080))
  {
    start_receive();
    // io_context 를 백그라운드 스레드에서 돌립니다
    worker_ = std::thread([this](){
      io_context_.run();
    });
  }

  ~EyePosNode() {
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
      auto j = json::parse(payload);
      float left_x = j.at("lepupil_x");
      float left_z = j.at("lepupil_y");
      float right_x = j.at("repupil_x");
      float right_z = j.at("repupil_y");
      float y = 1.0f;  // 테스트용 하드코드
      if(left_x == null || right_x == null || left_z == null || right_z == null || y == null) return;
      json out = { {"lefteye_x", left_x}, {"lefteye_z", left_z},{"righteye_x",right_x},{"righteye_z",right_z}, {"y", y} };
      RCLCPP_INFO(get_logger(), "Fused JSON: %s", out.dump().c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "JSON parse error: %s", e.what());
    }
  }

  asio::io_context io_context_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_ep_;
  std::array<char, 1024> buf_;
  std::thread worker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EyePosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
