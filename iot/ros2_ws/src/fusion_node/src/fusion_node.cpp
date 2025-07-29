#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class FusionNode : public rclcpp::Node {
public:
  FusionNode()
  : Node("fusion_node")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "eye_fused_json", 10,
      std::bind(&FusionNode::on_json, this, std::placeholders::_1)
    );
  }

private:
  void on_json(const std_msgs::msg::String::SharedPtr msg) {
    try {
      auto j = json::parse(msg->data);
      float lefteye_x = j.at("lefteye_x");
      float lefteye_z = j.at("lefteye_z");
      float righteye_x = j.at("righteye_x");
      float righteye_z = j.at("righteye_z");
      float y  = j.at("y");
      RCLCPP_INFO(get_logger(),
        "Received fused: L(%.2f,%.2f) R(%.2f,%.2f) y=%.2f",
        lx,lz, rx,rz, y
      );
      // TODO: 여기서 /cmd_pose 퍼블리시 등 다음 단계 작업
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Fusion JSON parse error: %s", e.what());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
