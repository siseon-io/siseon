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
#define PORT_NAME                "/dev/ttyUSB0"      // 다이나믹셀 연결 포트
#define BAUDRATE                 1000000             // 통신 속도
#define PROTOCOL_VERSION         2.0                 // 다이나믹셀 프로토콜 버전 (XM430는 v2.0)

// Control Table 주소 (XM430 기준)
#define ADDR_TORQUE_ENABLE       64
#define ADDR_GOAL_POSITION       116
#define ADDR_PRESENT_POSITION    132
#define ADDR_MOVING_SPEED        112   // Profile Velocity

// 위치 범위 (0~4095, 12비트)
#define DXL_MINIMUM_POSITION_VALUE   0
#define DXL_MAXIMUM_POSITION_VALUE   4095

// 이동 속도 설정 값 (0=최대속도, 25≈5.7rpm 정도)
#define DXL_MOVING_SPEED_VALUE       25

class ArmControlNode : public rclcpp::Node {
public:
  ArmControlNode() : Node("arm_control_node")
  {
    // 조인트 이름 ↔ ID (joint1~4가 각각 다이나믹셀 ID 11~14에 매핑)
    joint_names_ = {"joint1","joint2","joint3","joint4"};
    joint_to_id_ = { {"joint1",11},{"joint2",12},{"joint3",13},{"joint4",14} };

    // ---- 파라미터(보정/리밋) ----
    // 기본은 "캘리브 비활성 + 안전 리밋 활성"
    declare_parameter<bool>("use_calibration", false);
    declare_parameter<bool>("use_limits", true);

    // 선형 보정 q_corr = a*q_raw + b
    declare_parameter<double>("a_j11", 1.0); declare_parameter<double>("b_j11", 0.0);
    declare_parameter<double>("a_j12", 1.0); declare_parameter<double>("b_j12", 0.0);
    declare_parameter<double>("a_j13", 1.0); declare_parameter<double>("b_j13", 0.0);
    declare_parameter<double>("a_j14", 1.0); declare_parameter<double>("b_j14", 0.0);

    // 안전 리밋 (joint11은 정면 기준 ±π/4 로 제한)
    declare_parameter<double>("min_j11", -M_PI/4.0); declare_parameter<double>("max_j11", M_PI/4.0);
    declare_parameter<double>("min_j12",  0.00);     declare_parameter<double>("max_j12",  0.95);
    declare_parameter<double>("min_j13", -1.65);     declare_parameter<double>("max_j13",  1.10);
    declare_parameter<double>("min_j14", -1.20);     declare_parameter<double>("max_j14",  1.20);

    // 파라미터 로드 & 동적 파라미터 콜백 등록
    load_params_();
    cb_handle_ = add_on_set_parameters_callback(
      std::bind(&ArmControlNode::on_param_, this, std::placeholders::_1));

    // Dynamixel 연결/초기화 (포트/baud 설정, 토크 ON, 속도 설정)
    port_   = dynamixel::PortHandler::getPortHandler(PORT_NAME);
    packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!setup_()) {
      RCLCPP_ERROR(get_logger(), "Dynamixel init failed");
      rclcpp::shutdown();
      return;
    }

    // ROS I/F: 상태 퍼블리셔, 명령 구독, 주기 타이머
    pub_state_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_cmd_   = create_subscription<arm_control_node::msg::CmdPose>(
      "/cmd_pose", 10, std::bind(&ArmControlNode::on_cmd_, this, std::placeholders::_1));
    timer_     = create_wall_timer(100ms, std::bind(&ArmControlNode::on_timer_, this));

    RCLCPP_INFO(get_logger(), "🦾 arm_control_node up | calibration=%s limits=%s",
                use_calib_ ? "ON" : "OFF",
                use_limits_ ? "ON" : "OFF");
  }

  ~ArmControlNode() override {
    // 종료 시 모든 모터 토크 OFF
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
  bool use_calib_{false};  // 보정 사용 여부
  bool use_limits_{true};  // 안전 리밋 사용 여부
  std::array<double,4> a_{1.0,1.0,1.0,1.0};   // 보정 계수 a
  std::array<double,4> b_{0.0,0.0,0.0,0.0};   // 보정 오프셋 b
  std::array<double,4> qmin_{-M_PI/4.0, 0.00, -1.65, -1.20}; // 최소 각
  std::array<double,4> qmax_{ M_PI/4.0, 0.95,  1.10,  1.20}; // 최대 각

  // --- 유틸 ---
  // 라디안 값을 [-π, π] 범위로 래핑 (경계에서 점프 방지)
  static inline double wrap_pi(double x) {
    return std::atan2(std::sin(x), std::cos(x));  // [-π, π]로 안정 래핑
  }
  // 라디안 → 다이나믹셀 틱 ([-π,π]→[0..4095])
  static int rad2dxl_(double r){
    r = wrap_pi(r);  // 안전 래핑
    double s = (r + M_PI) / (2.0 * M_PI) * 4095.0;
    s = std::clamp(s, 0.0, 4095.0);
    return static_cast<int>(std::lround(s));
  }
  // 다이나믹셀 틱 → 라디안 ([0..4095]→[-π,π])
  static double dxl2rad_(uint32_t pos){
    return (static_cast<double>(pos - DXL_MINIMUM_POSITION_VALUE) / DXL_MAXIMUM_POSITION_VALUE)
           * 2.0 * M_PI - M_PI;
  }

  // 각도 보정 + 리밋 적용
  inline double apply(size_t i, double raw)
  {
    if (i == 0) {
      // joint11: 정면을 0 rad로 사용.
      //  - 경계 점프 방지를 위해 wrap, 그리고 안전 리밋(±π/4)만 적용.
      double delta = wrap_pi(raw);                             // 경계 안정화
      double lim   = std::min(std::abs(qmin_[0]), std::abs(qmax_[0])); // 대칭 리밋
      delta = std::clamp(delta, -lim, lim);                    // ±limit 제한
      return delta;                                            // center=0 이므로 그대로 반환
    }

    // ── 나머지 관절(12~14)은 (옵션)선형 보정 후, 리밋 적용 ──
    double v = use_calib_ ? (a_[i]*raw + b_[i]) : raw;
    if (use_limits_) v = std::clamp(v, qmin_[i], qmax_[i]);
    return v;
  }

  // 파라미터 로드 (노드 시작 시 호출)
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

  // 동적 파라미터 변경 콜백 (런타임에 rqt/CLI로 수정 시 반영)
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

  // /cmd_pose 콜백 — fusion_node가 보낸 목표 각도(q)를 수신하여 모터에 전송
  void on_cmd_(const arm_control_node::msg::CmdPose::SharedPtr m){
    // 지연(발행→수신) 측정 로그
    if (m->header.stamp.sec || m->header.stamp.nanosec){
      auto lat = now() - rclcpp::Time(m->header.stamp);
      RCLCPP_INFO(get_logger(), "Total Latency: %.2f ms", lat.seconds() * 1000.0);
    }

    // 수신한 라디안 각도(raw)
    std::array<double,4> q_raw{m->m11, m->m12, m->m13, m->m14};
    
    // 보정/리밋 적용
    std::array<double,4> q{
      apply(0, q_raw[0]),
      apply(1, q_raw[1]),
      apply(2, q_raw[2]),
      apply(3, q_raw[3])
    };
    
    // 각 조인트 라디안 → 다이나믹셀 위치값으로 변환 후 GOAL_POSITION에 기록
    for (const auto& it : std::vector<std::pair<std::string,double>>{
          {"joint1", q[0]}, {"joint2", q[1]}, {"joint3", q[2]}, {"joint4", q[3]} }){
      int id   = joint_to_id_[it.first];
      int goal = rad2dxl_(it.second);  // [-π,π] → [0..4095]
      uint8_t err = 0;
      int rc = packet_->write4ByteTxRx(port_, id, ADDR_GOAL_POSITION, goal, &err);
      if (rc != COMM_SUCCESS)
        RCLCPP_ERROR(get_logger(), "ID %d comm fail: %s", id, packet_->getTxRxResult(rc));
      else if (err)
        RCLCPP_ERROR(get_logger(), "ID %d hw err: %s",  id, packet_->getRxPacketError(err));
    }  

    // 디버그: 수신 각(q_raw)과 실제 전송 각(q)의 비교 로그
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
      "q_raw=[%.4f, %.4f, %.4f, %.4f] -> q_send=[%.4f, %.4f, %.4f, %.4f]",
      q_raw[0], q_raw[1], q_raw[2], q_raw[3], q[0], q[1], q[2], q[3]);
  }

  // 주기적으로 현재 모터 위치를 읽어 /joint_states 퍼블리시
  void on_timer_(){
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = now();
    for (const auto& n : joint_names_){
      int id = joint_to_id_[n];
      uint32_t pos = 0; uint8_t err = 0;
      int rc = packet_->read4ByteTxRx(port_, id, ADDR_PRESENT_POSITION, &pos, &err);
      if (rc == COMM_SUCCESS && !err) {
        msg->name.push_back(n);
        msg->position.push_back(dxl2rad_(pos)); // [0..4095] → [-π,π]
      }
    }
    if (!msg->name.empty()) pub_state_->publish(std::move(msg));
  }

  // 초기화: 포트/baud 설정 → 각 ID 토크 ON & 속도 설정
  bool setup_(){
    if (!port_->openPort()) { RCLCPP_ERROR(get_logger(),"openPort failed: %s", PORT_NAME); return false; }
    if (!port_->setBaudRate(BAUDRATE)) { RCLCPP_ERROR(get_logger(),"setBaudRate failed: %d", BAUDRATE); return false; }
    RCLCPP_INFO(get_logger(), "Port OK: %s @ %d", PORT_NAME, BAUDRATE);

    for (const auto& p : joint_to_id_){
      uint8_t e = 0;
      // Torque ON
      int rc = packet_->write1ByteTxRx(port_, p.second, ADDR_TORQUE_ENABLE, 1, &e);
      if (rc != COMM_SUCCESS || e) { RCLCPP_ERROR(get_logger(),"ID %d torque ON fail", p.second); return false; }
      // Profile Velocity (속도 설정)
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
