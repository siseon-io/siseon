#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "arm_control_node/msg/cmd_pose.hpp"

#include <string>
#include <cmath>
#include <vector>
#include <optional>

// --- 좌표 변환을 위한 튜닝 파라미터 ---
const double Y_LIDAR_SCALE = 0.01;      // LiDAR cm->m 변환
const double LIDAR_DISTANCE_OFFSET_M = 0.30; // 로봇팔과 목표물 사이의 안전 거리 (30cm)
const double MIN_FORWARD_REACH_M = 0.05;   // 로봇팔의 최소 전방 도달 거리 (5cm)

const double X_PIXEL_SCALE = 0.001;    // 눈의 X좌표(pixel)를 미터로 변환 (eye x -> robot y)
const double Z_PIXEL_SCALE = -0.001;    // 눈의 Z좌표(pixel)를 미터로 변환 (eye z -> robot z)
const double X_PIXEL_OFFSET = 320.0;    // 카메라 X축 중심 픽셀
const double Z_PIXEL_OFFSET = 350.0;    // 로봇 Z축 영점과 관련된 픽셀 오프셋

// OpenManipulator-X 로봇팔의 링크 길이 (단위: m)
const double LINK1_LENGTH = 0.077;
const double LINK2_LENGTH = 0.130;
const double LINK3_LENGTH = 0.124;
const double LINK4_LENGTH = 0.126;

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("fusion_node") {
        control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/control_mode", 10,
            std::bind(&FusionNode::control_mode_callback, this, std::placeholders::_1));

        eye_pose_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/eye_pose", 10,
            std::bind(&FusionNode::eye_pose_callback, this, std::placeholders::_1));

        manual_pose_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/manual_pose", 10,
            std::bind(&FusionNode::manual_pose_callback, this, std::placeholders::_1));

        preset_pose_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/preset_pose", 10,
            std::bind(&FusionNode::preset_pose_callback, this, std::placeholders::_1));

        cmd_pose_pub_ = this->create_publisher<arm_control_node::msg::CmdPose>("/cmd_pose", 10);

        current_mode_ = "auto";
        // RCLCPP_INFO(this->get_logger(), "🚀 Fusion node started. Default mode: 'auto'");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eye_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr manual_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr preset_pose_sub_;
    rclcpp::Publisher<arm_control_node::msg::CmdPose>::SharedPtr cmd_pose_pub_;

    std::string current_mode_;
    geometry_msgs::msg::Point last_eye_pose_;
    geometry_msgs::msg::Point last_manual_pose_;
    geometry_msgs::msg::Point last_preset_pose_;

    std::optional<double> parse_json_value(const std::string& json_str, const std::string& key) {
        std::string search_key = "\"" + key + "\":";
        size_t key_pos = json_str.find(search_key);
        if (key_pos == std::string::npos) return std::nullopt;

        size_t value_start = key_pos + search_key.length();
        size_t value_end = json_str.find_first_of(",}", value_start);
        if (value_end == std::string::npos) return std::nullopt;

        std::string value_str = json_str.substr(value_start, value_end - value_start);
        try {
            return std::stod(value_str);
        } catch (const std::exception& e) {
            // RCLCPP_ERROR(this->get_logger(), "Failed to parse value for key '%s': %s", key.c_str(), e.what());
            return std::nullopt;
        }
    }
    
    std::optional<std::vector<double>> calculate_ik(const geometry_msgs::msg::Point& target_pose) {
        double px = -target_pose.x;
        double py = -target_pose.y;
        double pz = target_pose.z;

        double q1 = atan2(py, px);
        double r = sqrt(pow(px, 2) + pow(py, 2));
        double wrist_x = r;
        double wrist_z = pz + LINK4_LENGTH;
        double rel_wrist_x = wrist_x;
        double rel_wrist_z = wrist_z - LINK1_LENGTH;
        double dist_sq = pow(rel_wrist_x, 2) + pow(rel_wrist_z, 2);
        double dist = sqrt(dist_sq);

        if (dist > LINK2_LENGTH + LINK3_LENGTH || dist < std::abs(LINK2_LENGTH - LINK3_LENGTH)) {
            // RCLCPP_WARN(this->get_logger(), "Unreachable position: (%.2f, %.2f, %.2f)", px, py, pz);
            return std::nullopt;
        }

        double cos_q3 = (dist_sq - pow(LINK2_LENGTH, 2) - pow(LINK3_LENGTH, 2)) / (2 * LINK2_LENGTH * LINK3_LENGTH);
        cos_q3 = std::max(-1.0, std::min(1.0, cos_q3));
        double q3 = -acos(cos_q3);
        double q2 = atan2(rel_wrist_z, rel_wrist_x) - atan2(LINK3_LENGTH * sin(q3), LINK2_LENGTH + LINK3_LENGTH * cos(q3));
        double q4 = -M_PI / 2.0 - q2 - q3;

        return std::vector<double>{q1, q2, q3, q4};
    }

    void control_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        current_mode_ = msg->data;
        // RCLCPP_INFO(this->get_logger(), "✅ control_mode updated: %s", current_mode_.c_str());
        publish_latest_pose();
    }

    void eye_pose_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto lx = parse_json_value(msg->data, "lefteye_x");
        auto lz = parse_json_value(msg->data, "lefteye_z");
        auto rx = parse_json_value(msg->data, "righteye_x");
        auto rz = parse_json_value(msg->data, "righteye_z");
        auto y = parse_json_value(msg->data, "y");

        if (!lx || !lz || !rx || !rz || !y) {
            // RCLCPP_WARN(this->get_logger(), "Failed to parse all values from eye_pose JSON");
            return;
        }
        
        double avg_x_px = (*lx + *rx) / 2.0;
        double avg_z_px = (*lz + *rz) / 2.0;
        double y_cm = *y;
        
        // ★★★ 목표 x 좌표 계산 로직 수정 ★★★
        // 1. LiDAR 거리를 미터 단위로 변환
        double lidar_dist_m = y_cm * Y_LIDAR_SCALE;
        // 2. LiDAR 거리에서 오프셋(30cm)을 뺀 값을 목표로 설정
        double target_x = lidar_dist_m - LIDAR_DISTANCE_OFFSET_M;
        // 3. 목표가 너무 가깝거나 음수가 되지 않도록 최소 도달 거리 적용
        last_eye_pose_.x = std::max(MIN_FORWARD_REACH_M, target_x);
        
        // 로봇 y, z 좌표계로 변환
        last_eye_pose_.y = (avg_x_px - X_PIXEL_OFFSET) * X_PIXEL_SCALE;
        last_eye_pose_.z = (avg_z_px - Z_PIXEL_OFFSET) * Z_PIXEL_SCALE;
        
        if (current_mode_ == "auto") {
            publish_latest_pose();
        }
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
        geometry_msgs::msg::Point pose_to_process;
        std::string mode_info;

        if (current_mode_ == "auto") {
            pose_to_process = last_eye_pose_;
            mode_info = "AUTO";
        }
        else if (current_mode_ == "manual") {
            pose_to_process = last_manual_pose_;
            mode_info = "MANUAL";
        }
        else if (current_mode_ == "preset") {
            pose_to_process = last_preset_pose_;
            mode_info = "PRESET";
        }
        else {
            // RCLCPP_WARN(this->get_logger(), "⚠️ Unknown control_mode: %s", current_mode_.c_str());
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "[%s Mode] Processing pose: (x:%.2f, y:%.2f, z:%.2f)",
        //             mode_info.c_str(), pose_to_process.x, pose_to_process.y, pose_to_process.z);

        auto motor_angles = calculate_ik(pose_to_process);

        if (motor_angles) {
            arm_control_node::msg::CmdPose motor_cmd;
            motor_cmd.m11 = motor_angles->at(0);
            motor_cmd.m12 = motor_angles->at(1);
            motor_cmd.m13 = motor_angles->at(2);
            motor_cmd.m14 = motor_angles->at(3);

            cmd_pose_pub_->publish(motor_cmd);

            // RCLCPP_INFO(this->get_logger(), "→ Publishing cmd_pose: [%.2f, %.2f, %.2f, %.2f]",
                        // motor_cmd.m11, motor_cmd.m12, motor_cmd.m13, motor_cmd.m14);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}