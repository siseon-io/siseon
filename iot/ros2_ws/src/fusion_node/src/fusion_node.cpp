#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "arm_control_node/msg/cmd_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/qos.hpp"

#include <string>
#include <cmath>
#include <vector>
#include <optional>
#include <chrono>
#include <memory>
#include <numeric>

using namespace std::chrono_literals;

// --- 데이터 기반 제어를 위한 새 구조체 ---

// 3D 좌표와 4개의 관절 각도를 묶는 구조체
struct CalibrationPoint {
    geometry_msgs::msg::Point eye_pos; // 카메라 좌표계 기준 눈의 위치 (mm)
    std::vector<double> joint_angles;  // 해당 위치에 도달하기 위한 관절 각도 (radian)
};

// --- 제어 관련 설정 ---
struct ControlParameters {
    double move_threshold_rad = 0.01;      // 이 각도 미만의 움직임은 무시 (약 0.5도)
    double max_step_size_rad = 0.05;       // 한 제어 루프당 최대 이동 각도 (속도 제한)
    double idw_power = 2.0;                // 역거리 가중법의 가중치 파워 (클수록 가까운 점의 영향력이 커짐)
};

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("fusion_node") {
        auto cmd_qos = rclcpp::QoS(1).transient_local();
        cmd_pose_pub_ = this->create_publisher<arm_control_node::msg::CmdPose>("/cmd_pose", cmd_qos);

        control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/control_mode", 10, std::bind(&FusionNode::control_mode_callback, this, std::placeholders::_1));
        
        eye_pose_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/eye_pose", 10, std::bind(&FusionNode::eye_pose_callback, this, std::placeholders::_1));

        // 제공된 보정 데이터를 여기에 저장합니다.
        initialize_calibration_data();

        control_loop_timer_ = this->create_wall_timer(
            50ms, std::bind(&FusionNode::control_loop, this)); // 루프 주기를 50ms로 줄여 반응성 향상

        RCLCPP_INFO(this->get_logger(), "🚀 Data-driven Fusion node started.");
    }

private:
    // --- ROS 관련 멤버 ---
    rclcpp::Publisher<arm_control_node::msg::CmdPose>::SharedPtr cmd_pose_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eye_pose_sub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    
    // --- 상태 및 데이터 멤버 ---
    ControlParameters control_params_;
    std::string current_mode_ = "fix"; // 기본 모드는 fix
    std::vector<CalibrationPoint> calibration_data_;
    
    // 가장 최근에 계산된 목표 눈 위치 (카메라 좌표계)
    std::optional<geometry_msgs::msg::Point> latest_target_eye_pos_ = std::nullopt;
    rclcpp::Time last_eye_data_time_;

    // 가장 최근에 발행된 관절 각도
    std::vector<double> last_commanded_angles_{0.0, 0.0, 0.0, 0.0};
    uint64_t latest_source_timestamp_ns_ = 0;

    // --- 유틸리티 함수 ---

    // JSON 문자열에서 vector<double> 파싱
    std::optional<std::vector<double>> parse_json_vector(const std::string& json_str, const std::string& key) {
        std::string search_key = "\"" + key + "\":[";
        size_t key_pos = json_str.find(search_key);
        if (key_pos == std::string::npos) return std::nullopt;
        size_t value_start = key_pos + search_key.length();
        size_t value_end = json_str.find("]", value_start);
        if (value_end == std::string::npos) return std::nullopt;
        std::string arr_str = json_str.substr(value_start, value_end - value_start);

        std::vector<double> values;
        std::stringstream ss(arr_str);
        std::string item;
        while(std::getline(ss, item, ',')) {
            try {
                values.push_back(std::stod(item));
            } catch (const std::exception&) { return std::nullopt; }
        }
        return values;
    }
    
    // JSON 문자열에서 uint64_t 파싱
    std::optional<uint64_t> parse_json_uint64(const std::string& json_str, const std::string& key) {
        std::string search_key = "\"" + key + "\":";
        size_t key_pos = json_str.find(search_key);
        if (key_pos == std::string::npos) return std::nullopt;
        size_t value_start = key_pos + search_key.length();
        size_t value_end = json_str.find_first_of(",}", value_start);
        if (value_end == std::string::npos) return std::nullopt;
        std::string value_str = json_str.substr(value_start, value_end - value_start);
        try { return std::stoull(value_str); } catch (const std::exception&) { return std::nullopt; }
    }

    // --- 핵심 로직 함수 ---

    void initialize_calibration_data() {
        calibration_data_.clear();
        CalibrationPoint p;

        // UP
        p.eye_pos.x = 133.39; p.eye_pos.y = -162.67; p.eye_pos.z = 1028.0;
        p.joint_angles = {-3.0971, 0.5706, -1.4067, 1.4711};
        calibration_data_.push_back(p);

        // DOWN
        p.eye_pos.x = 139.70; p.eye_pos.y = 86.42; p.eye_pos.z = 850.5;
        p.joint_angles = {-3.1109, 0.3206, -0.4740, 1.0186};
        calibration_data_.push_back(p);

        // LEFT
        p.eye_pos.x = 421.61; p.eye_pos.y = -131.52; p.eye_pos.z = 912.0;
        p.joint_angles = {2.8256, 0.2592, -0.6550, 0.9235};
        calibration_data_.push_back(p);

        // RIGHT
        p.eye_pos.x = -296.75; p.eye_pos.y = -174.36; p.eye_pos.z = 774.0;
        p.joint_angles = {-2.5127, 0.5047, -0.8115, 0.8406};
        calibration_data_.push_back(p);

        // FRONT
        p.eye_pos.x = 159.17; p.eye_pos.y = -130.72; p.eye_pos.z = 977.0;
        p.joint_angles = {-3.1094, 0.0522, -0.6151, 1.0815};
        calibration_data_.push_back(p);

        // BACK
        p.eye_pos.x = 251.11; p.eye_pos.y = -249.07; p.eye_pos.z = 895.5;
        p.joint_angles = {-3.1247, 0.7302, -0.1212, -0.2148};
        calibration_data_.push_back(p);

        RCLCPP_INFO(this->get_logger(), "✅ Calibration data initialized with %zu points.", calibration_data_.size());
    }

    // 새로운 관절 각도 계산 함수 (IDW 보간)
    std::optional<std::vector<double>> calculate_joint_angles_from_calibration(const geometry_msgs::msg::Point& target_pos) {
        std::vector<double> distances;
        std::vector<double> weights;
        double total_weight = 0.0;

        for (const auto& p : calibration_data_) {
            double dist = std::sqrt(std::pow(target_pos.x - p.eye_pos.x, 2) +
                                    std::pow(target_pos.y - p.eye_pos.y, 2) +
                                    std::pow(target_pos.z - p.eye_pos.z, 2));
            // 거리가 0에 매우 가까우면, 해당 점의 각도를 그대로 사용 (100% 가중치)
            if (dist < 1e-6) {
                return p.joint_angles;
            }
            distances.push_back(dist);
        }

        for (double dist : distances) {
            double weight = 1.0 / std::pow(dist, control_params_.idw_power);
            weights.push_back(weight);
            total_weight += weight;
        }

        std::vector<double> final_angles(4, 0.0);
        for (size_t i = 0; i < calibration_data_.size(); ++i) {
            for (size_t j = 0; j < 4; ++j) {
                final_angles[j] += calibration_data_[i].joint_angles[j] * (weights[i] / total_weight);
            }
        }
        return final_angles;
    }

    // --- ROS 콜백 및 루프 ---

    void control_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Mode changed from '%s' to '%s'", current_mode_.c_str(), msg->data.c_str());
        current_mode_ = msg->data;
        if(current_mode_ != "auto"){
            latest_target_eye_pos_ = std::nullopt; // auto 모드가 아니면 목표 지점 초기화
        }
    }

    void eye_pose_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto pose_xyz_opt = parse_json_vector(msg->data, "pose_xyz");
        auto timestamp_ns = parse_json_uint64(msg->data, "timestamp");

        if (!timestamp_ns || !pose_xyz_opt || pose_xyz_opt->size() != 3) {
            return;
        }
        
        geometry_msgs::msg::Point pos;
        pos.x = (*pose_xyz_opt)[0];
        pos.y = (*pose_xyz_opt)[1];
        pos.z = (*pose_xyz_opt)[2];

        // 유효하지 않은 데이터(0,0,0)는 건너뜀
        if (pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0) {
            return;
        }

        latest_target_eye_pos_ = pos;
        latest_source_timestamp_ns_ = *timestamp_ns;
        last_eye_data_time_ = this->now();
    }

    void control_loop() {
        if (current_mode_ != "auto") return;

        if (!latest_target_eye_pos_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for valid eye target...");
            return;
        }

        if ((this->now() - last_eye_data_time_).seconds() > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Eye data is stale. Holding position.");
            return;
        }

        auto target_angles_opt = calculate_joint_angles_from_calibration(*latest_target_eye_pos_);
        if (!target_angles_opt) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not calculate target angles.");
            return;
        }

        auto target_angles = *target_angles_opt;

        // 현재 각도와 목표 각도의 차이가 임계값보다 작으면 무시
        double diff = 0.0;
        for(size_t i=0; i<4; ++i) {
            diff += std::pow(target_angles[i] - last_commanded_angles_[i], 2);
        }
        if (std::sqrt(diff) < control_params_.move_threshold_rad && last_commanded_angles_[0] != 0.0) {
             return; // 이미 목표 근처에 도달했으면 명령 발행 안함
        }

        // 최대 속도 제한 적용 (Slerp와 유사한 간단한 선형 보간)
        for(size_t i=0; i<4; ++i) {
            double step = target_angles[i] - last_commanded_angles_[i];
            if (std::abs(step) > control_params_.max_step_size_rad) {
                target_angles[i] = last_commanded_angles_[i] + (step > 0 ? 1 : -1) * control_params_.max_step_size_rad;
            }
        }
        
        arm_control_node::msg::CmdPose motor_cmd;
        motor_cmd.header.stamp = rclcpp::Time(latest_source_timestamp_ns_);
        motor_cmd.m11 = target_angles[0];
        motor_cmd.m12 = target_angles[1];
        motor_cmd.m13 = target_angles[2];
        motor_cmd.m14 = target_angles[3];
        
        cmd_pose_pub_->publish(motor_cmd);
        last_commanded_angles_ = target_angles;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}