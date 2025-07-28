// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <nlohmann/json.hpp>
// #include <websocketpp/config/asio_no_tls_client.hpp>
// #include <websocketpp/client.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <cmath>

// using websocketpp::connection_hdl;
// using json = nlohmann::json;

// class EyePosNode : public rclcpp::Node {
// public:
//   EyePosNode()
//   : Node("eye_pos_node"), latest_lidar_(std::nanf("1")) {
//     // // LiDAR 구독
//     // lidar_sub_ = this->create_subscription<std_msgs::msg::Float32>(
//     //   "/lidar_dist", 10,
//     //   std::bind(&EyePosNode::lidarCallback, this, std::placeholders::_1)
//     // );
//     // 웹소켓 클라이언트
//     client_.init_asio();
//     client_.set_message_handler(
//       std::bind(&EyePosNode::onWebsocket, this, std::placeholders::_1, std::placeholders::_2)
//     );
//     client_.start_perpetual();
//     ws_thread_ = std::thread([this](){
//       auto con = client_.get_connection("ws://10.0.0.2:30080", ec_);
//       client_.connect(con);
//       client_.run();
//     });
//   }
//   ~EyePosNode() {
//     client_.stop_perpetual();
//     ws_thread_.join();
//   }
// private:
//   void lidarCallback(const std_msgs::msg::Float32::SharedPtr msg) {
//     // latest_lidar_ = msg->data;
//     latest_lidar_ = 1;
//   }
//   void onWebsocket(connection_hdl, websocketpp::config::asio_client::message_type::ptr m) {
//     try {
//     //   auto j = json::parse(m->get_payload());
//     //   float x = j.at("x");
//     //   float z = j.at("z");
//     //   if (std::isnan(latest_lidar_)) return;
//     //   json out = { {"x", 1}, {"y", latest_lidar_}, {"z", 1} };
//     //   RCLCPP_INFO(this->get_logger(), "Fused JSON: %s", out.dump().c_str());
//     json out = { {"x",1}, {"y",1}, {"z",1} };
//     RCLCPP_INFO(this->get_logger(), "Fused JSON: %s", out.dump().c_str());
//     } catch(const std::exception &e) {
//       RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
//     }
//   }
// //   rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lidar_sub_;
// //   float latest_lidar_;
//   websocketpp::client<websocketpp::config::asio_client> client_;
//   websocketpp::lib::error_code ec_;
//   std::thread ws_thread_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<EyePosNode>());
//   rclcpp::shutdown();
//   return 0;
// }

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
      float x = j.at("x");
      float z = j.at("z");
      float y = 1.0f;  // 테스트용 하드코딩
      json out = { {"x", x}, {"y", y}, {"z", z} };
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
