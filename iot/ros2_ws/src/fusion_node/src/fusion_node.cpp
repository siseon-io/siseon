#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <unordered_map>
#include <string>

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("fusion_node") {
        control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/control_mode", 10,
            std::bind(&FusionNode::control_mode_callback, this, std::placeholders::_1));

        eye_pose_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/eye_pose", 10,
            std::bind(&FusionNode::eye_pose_callback, this, std::placeholders::_1));

        manual_pose_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/manual_pose", 10,
            std::bind(&FusionNode::manual_pose_callback, this, std::placeholders::_1));

        preset_pose_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/preset_pose", 10,
            std::bind(&FusionNode::preset_pose_callback, this, std::placeholders::_1));

        cmd_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_pose", 10);

        current_mode_ = "preset";
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr eye_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr manual_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr preset_pose_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pose_pub_;

    std::string current_mode_;
    geometry_msgs::msg::Point last_eye_pose_;
    geometry_msgs::msg::Point last_manual_pose_;
    geometry_msgs::msg::Point last_preset_pose_;

    void control_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        current_mode_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "\u2705 control_mode 갱신: %s", current_mode_.c_str());
        publish_latest_pose();
    }

    void eye_pose_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        last_eye_pose_ = *msg;
        if (current_mode_ == "auto") publish_latest_pose();
    }

    void manual_pose_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        last_manual_pose_ = *msg;
        if (current_mode_ == "manual") publish_latest_pose();
    }

    void preset_pose_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        last_preset_pose_ = *msg;
        if (current_mode_ == "preset") publish_latest_pose();
    }

    void publish_latest_pose() {
        geometry_msgs::msg::Point pose;

        if (current_mode_ == "auto") pose = last_eye_pose_;
        else if (current_mode_ == "manual") pose = last_manual_pose_;
        else if (current_mode_ == "preset") pose = last_preset_pose_;
        else {
            RCLCPP_WARN(this->get_logger(), "\u26a0\ufe0f 알 수 없는 control_mode: %s", current_mode_.c_str());
            return;
        }

        std_msgs::msg::Float64MultiArray motor_cmd;
        motor_cmd.data.resize(5);

        // 임시 모터 변환 로직 (예시)
        motor_cmd.data[0] = pose.x * 10.0; // m1
        motor_cmd.data[1] = pose.y * 10.0; // m2
        motor_cmd.data[2] = pose.z * 10.0; // m3
        motor_cmd.data[3] = pose.x + pose.y; // m4
        motor_cmd.data[4] = pose.y + pose.z; // m5

        cmd_pose_pub_->publish(motor_cmd);

        RCLCPP_INFO(this->get_logger(), "\u2192 cmd_pose 퍼블리시: [%.2f, %.2f, %.2f, %.2f, %.2f]",
                    motor_cmd.data[0], motor_cmd.data[1], motor_cmd.data[2],
                    motor_cmd.data[3], motor_cmd.data[4]);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}