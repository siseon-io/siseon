// arm_control_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "arm_control_node/msg/cmd_pose.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <array>
#include <algorithm>
#include <chrono>
using namespace std::chrono_literals;

// --- Dynamixel (XM430) 기본 설정 ---
#define PORT_NAME                "/dev/ttyUSB0"
#define BAUDRATE                 1000000
#define PROTOCOL_VERSION         2.0

// Control Table 주소
#define ADDR_TORQUE_ENABLE       64
#define ADDR_GOAL_POSITION       116
#define ADDR_PRESENT_POSITION    132
#define ADDR_MOVING_SPEED        112   // Profile Velocity

// 위치 범위
#define DXL_MINIMUM_POSITION_VALUE   0
#define DXL_MAXIMUM_POSITION_VALUE   4095

// 이동 속도 설정 값 (0=최대, 25≈5.7rpm)
#define DXL_MOVING_SPEED_VALUE       25

class ArmControlNode : public rclcpp::Node {
public:
  ArmControlNode() : Node("arm_control_node")
  {
    // 조인트 이름 ↔ ID
    joint_names_ = {"joint1","joint2","joint3","joint4"};
    joint_to_id_ = { {"joint1",11},{"joint2",12},{"joint3",13},{"joint4",14} };

    // ---- 파라미터(캘리브/리밋) ----
    // 기본은 "그대로 전달" + 안전 리밋만 적용
    declare_parameter<bool>("use_calibration", false);
    declare_parameter<bool>("use_limits", true);

    declare_parameter<double>("a_j11", 1.0); declare_parameter<double>("b_j11", 0.0);
    declare_parameter<double>("a_j12", 1.0); declare_parameter<double>("b_j12", 0.0);
    declare_parameter<double>("a_j13", 1.0); declare_parameter<double>("b_j13", 0.0);
    declare_parameter<double>("a_j14", 1.0); declare_parameter<double>("b_j14", 0.0);

    // 안전 리밋 (필요시 조정)
    declare_parameter<double>("min_j11", -M_PI); declare_parameter<double>("max_j11",  M_PI);
    declare_parameter<double>("min_j12",  0.00); declare_parameter<double>("max_j12",  0.95);
    declare_parameter<double>("min_j13", -1.65); declare_parameter<double>("max_j13",  1.10);
    declare_parameter<double>("min_j14", -1.20); declare_parameter<double>("max_j14",  1.20);

    load_params_();
    cb_handle_ = add_on_set_parameters_callback(
      std::bind(&ArmControlNode::on_param_, this, std::placeholders::_1));

    // Dynamixel 연결/초기화 (간단 로직)
    port_   = dynamixel::PortHandler::getPortHandler(PORT_NAME);
    packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!setup_()) {
      RCLCPP_ERROR(get_logger(), "Dynamixel init failed");
      rclcpp::shutdown();
      return;
    }

    // ROS I/F
    pub_state_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_cmd_   = create_subscription<arm_control_node::msg::CmdPose>(
      "/cmd_pose", 10, std::bind(&ArmControlNode::on_cmd_, this, std::placeholders::_1));
    timer_     = create_wall_timer(100ms, std::bind(&ArmControlNode::on_timer_, this));

    RCLCPP_INFO(get_logger(), "🦾 arm_control_node up | calibration=%s limits=%s",
                use_calib_ ? "ON" : "OFF",
                use_limits_ ? "ON" : "OFF");
  }

  ~ArmControlNode() override {
    RCLCPP_INFO(get_logger(), "Disabling torque...");
    for (const auto& p : joint_to_id_) {
      packet_->write1ByteTxRx(port_, p.second, ADDR_TORQUE_ENABLE, 0);
    }
    port_->closePort();
  }

private:
  // --- ROS ---
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_state_;
  rclcpp::Subscription<arm_control_node::msg::CmdPose>::SharedPtr sub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;

  // --- Dynamixel ---
  dynamixel::PortHandler*  port_{nullptr};
  dynamixel::PacketHandler* packet_{nullptr};

  std::map<std::string,int>  joint_to_id_;
  std::vector<std::string>   joint_names_;

  // --- 파라미터 상태 ---
  bool use_calib_{false};
  bool use_limits_{true};
  std::array<double,4> a_{1.0,1.0,1.0,1.0};
  std::array<double,4> b_{0.0,0.0,0.0,0.0};
  std::array<double,4> qmin_{-M_PI, 0.00, -1.65, -1.20};
  std::array<double,4> qmax_{ M_PI, 0.95,  1.10,  1.20};

  // --- 유틸 ---
  static inline double wrap_pi(double x) {
    return std::atan2(std::sin(x), std::cos(x));  // [-π, π]로 안정 래핑
  }
  static int rad2dxl_(double r){
    r = wrap_pi(r);  // 안전 래핑
    double s = (r + M_PI) / (2.0 * M_PI) * 4095.0;
    s = std::clamp(s, 0.0, 4095.0);
    return static_cast<int>(std::lround(s));
  }
  static double dxl2rad_(uint32_t pos){
    return (static_cast<double>(pos - DXL_MINIMUM_POSITION_VALUE) / DXL_MAXIMUM_POSITION_VALUE)
           * 2.0 * M_PI - M_PI;
  }
  
  inline double apply(size_t i, double raw)
  {
  if (i == 0) {
    // ── joint 11: 기준각 π(180°)를 중심으로 ±π/4 제한 ──
    const double center = M_PI;
    // (raw - π)를 [-π,π]로 래핑해서 경계점에서 점프 방지
    double delta = wrap_pi(raw - center);

    // (옵션) 보정이 켜져 있으면 delta에만 보정 적용
    if (use_calib_) delta = a_[0]*delta + b_[0];

    // ±π/4로 제한
    const double lim = M_PI / 4.0;
    delta = std::clamp(delta, -lim, lim);

    // 최종 값은 center(π) 기준으로 되돌리고 한 번 더 래핑
    double v = wrap_pi(center + delta);
    return v;  // joint11은 항상 이 제한을 따르게 함
  }

  // ── 나머지 관절(12~14)은 기존 로직 유지 ──
  double v = use_calib_ ? (a_[i]*raw + b_[i]) : raw;
  if (use_limits_) v = std::clamp(v, qmin_[i], qmax_[i]);
  return v;
  }

  void load_params_(){
    use_calib_ = get_parameter("use_calibration").as_bool();
    use_limits_ = get_parameter("use_limits").as_bool();

    a_[0] = get_parameter("a_j11").as_double(); b_[0] = get_parameter("b_j11").as_double();
    a_[1] = get_parameter("a_j12").as_double(); b_[1] = get_parameter("b_j12").as_double();
    a_[2] = get_parameter("a_j13").as_double(); b_[2] = get_parameter("b_j13").as_double();
    a_[3] = get_parameter("a_j14").as_double(); b_[3] = get_parameter("b_j14").as_double();

    qmin_[0] = get_parameter("min_j11").as_double(); qmax_[0] = get_parameter("max_j11").as_double();
    qmin_[1] = get_parameter("min_j12").as_double(); qmax_[1] = get_parameter("max_j12").as_double();
    qmin_[2] = get_parameter("min_j13").as_double(); qmax_[2] = get_parameter("max_j13").as_double();
    qmin_[3] = get_parameter("min_j14").as_double(); qmax_[3] = get_parameter("max_j14").as_double();
  }

  rcl_interfaces::msg::SetParametersResult
  on_param_(const std::vector<rclcpp::Parameter>& ps){
    for (const auto& p : ps){
      const auto& n = p.get_name();
      if      (n == "use_calibration") use_calib_  = p.as_bool();
      else if (n == "use_limits")      use_limits_ = p.as_bool();
      else if (n == "a_j11") a_[0] = p.as_double(); else if (n == "b_j11") b_[0] = p.as_double();
      else if (n == "a_j12") a_[1] = p.as_double(); else if (n == "b_j12") b_[1] = p.as_double();
      else if (n == "a_j13") a_[2] = p.as_double(); else if (n == "b_j13") b_[2] = p.as_double();
      else if (n == "a_j14") a_[3] = p.as_double(); else if (n == "b_j14") b_[3] = p.as_double();
      else if (n == "min_j11") qmin_[0] = p.as_double(); else if (n == "max_j11") qmax_[0] = p.as_double();
      else if (n == "min_j12") qmin_[1] = p.as_double(); else if (n == "max_j12") qmax_[1] = p.as_double();
      else if (n == "min_j13") qmin_[2] = p.as_double(); else if (n == "max_j13") qmax_[2] = p.as_double();
      else if (n == "min_j14") qmin_[3] = p.as_double(); else if (n == "max_j14") qmax_[3] = p.as_double();
    }
    rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
  }

  // /cmd_pose 콜백 — joint1~4 항상 전송
  void on_cmd_(const arm_control_node::msg::CmdPose::SharedPtr m){
    if (m->header.stamp.sec || m->header.stamp.nanosec){
      auto lat = now() - rclcpp::Time(m->header.stamp);
      RCLCPP_INFO(get_logger(), "Total Latency: %.2f ms", lat.seconds() * 1000.0);
    }

    std::array<double,4> q_raw{m->m11, m->m12, m->m13, m->m14};
    
    std::array<double,4> q{
      apply(0, q_raw[0]),
      apply(1, q_raw[1]),
      apply(2, q_raw[2]),
      apply(3, q_raw[3])
    };

    for (const auto& it : std::vector<std::pair<std::string,double>>{
          {"joint1", q[0]}, {"joint2", q[1]}, {"joint3", q[2]}, {"joint4", q[3]} }){
      int id   = joint_to_id_[it.first];
      int goal = rad2dxl_(it.second);
      uint8_t err = 0;
      int rc = packet_->write4ByteTxRx(port_, id, ADDR_GOAL_POSITION, goal, &err);
      if (rc != COMM_SUCCESS)
        RCLCPP_ERROR(get_logger(), "ID %d comm fail: %s", id, packet_->getTxRxResult(rc));
      else if (err)
        RCLCPP_ERROR(get_logger(), "ID %d hw err: %s",  id, packet_->getRxPacketError(err));
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
      "q_raw=[%.4f, %.4f, %.4f, %.4f] -> q_send=[%.4f, %.4f, %.4f, %.4f]",
      q_raw[0], q_raw[1], q_raw[2], q_raw[3], q[0], q[1], q[2], q[3]);
  }

  // 주기적으로 현재 위치 읽어서 /joint_states 퍼블리시
  void on_timer_(){
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = now();
    for (const auto& n : joint_names_){
      int id = joint_to_id_[n];
      uint32_t pos = 0; uint8_t err = 0;
      int rc = packet_->read4ByteTxRx(port_, id, ADDR_PRESENT_POSITION, &pos, &err);
      if (rc == COMM_SUCCESS && !err) {
        msg->name.push_back(n);
        msg->position.push_back(dxl2rad_(pos));
      }
    }
    if (!msg->name.empty()) pub_state_->publish(std::move(msg));
  }

  // 간단 초기화: 포트/baud → 각 ID torque ON & 속도 설정
  bool setup_(){
    if (!port_->openPort()) { RCLCPP_ERROR(get_logger(),"openPort failed: %s", PORT_NAME); return false; }
    if (!port_->setBaudRate(BAUDRATE)) { RCLCPP_ERROR(get_logger(),"setBaudRate failed: %d", BAUDRATE); return false; }
    RCLCPP_INFO(get_logger(), "Port OK: %s @ %d", PORT_NAME, BAUDRATE);

    for (const auto& p : joint_to_id_){
      uint8_t e = 0;
      // Torque ON
      int rc = packet_->write1ByteTxRx(port_, p.second, ADDR_TORQUE_ENABLE, 1, &e);
      if (rc != COMM_SUCCESS || e) { RCLCPP_ERROR(get_logger(),"ID %d torque ON fail", p.second); return false; }
      // Profile Velocity
      rc = packet_->write4ByteTxRx(port_, p.second, ADDR_MOVING_SPEED, DXL_MOVING_SPEED_VALUE, &e);
      if (rc != COMM_SUCCESS || e) { RCLCPP_ERROR(get_logger(),"ID %d set speed fail", p.second); return false; }
    }
    return true;
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

