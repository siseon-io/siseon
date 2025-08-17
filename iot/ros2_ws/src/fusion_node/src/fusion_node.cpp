// // fusion_node.cpp
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "arm_control_node/msg/cmd_pose.hpp"

// #include <optional>
// #include <string>
// #include <cmath>
// #include <array>
// #include <algorithm>
// #include <fstream>
// #include <sstream>

// using namespace std::chrono_literals;

// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif

// // =========================
// // Helpers / Math
// // =========================
// static inline double wrap_to_pi(double a) {
//   while (a >  M_PI) a -= 2.0 * M_PI;
//   while (a < -M_PI) a += 2.0 * M_PI;
//   return a;
// }

// static inline double slerp_angle(double a, double b, double t) {
//   double da = wrap_to_pi(b - a);
//   t = std::clamp(t, 0.0, 1.0);
//   return wrap_to_pi(a + da * t);
// }

// static inline std::string join_paths(const std::string& a, const std::string& b) {
//   if (a.empty()) return b;
//   return (a.back() == '/') ? (a + b) : (a + "/" + b);
// }

// // =========================
// // Tiny JSON helpers (ad-hoc)
// // =========================
// static std::optional<double> pick_double(const std::string& s, const std::string& key) {
//   const std::string tag = "\"" + key + "\":";
//   size_t p = s.find(tag);
//   if (p == std::string::npos) return std::nullopt;
//   p += tag.size();
//   size_t e = s.find_first_of(",}", p);
//   if (e == std::string::npos) return std::nullopt;
//   return std::stod(s.substr(p, e - p));
// }

// static std::optional<uint64_t> pick_u64(const std::string& s, const std::string& key) {
//   const std::string tag = "\"" + key + "\":";
//   size_t p = s.find(tag);
//   if (p == std::string::npos) return std::nullopt;
//   p += tag.size();
//   size_t e = s.find_first_of(",}", p);
//   if (e == std::string::npos) return std::nullopt;
//   return static_cast<uint64_t>(std::stoull(s.substr(p, e - p)));
// }

// // JSON: {"rad":[r11,r12,r13,r14]} 형태에서 rad 4개를 파싱
// static bool load_rad4_from_json(const std::string& path, std::array<double,4>& out) {
//   std::ifstream f(path);
//   if (!f.is_open()) return false;
//   std::stringstream buf; buf << f.rdbuf();
//   const std::string s = buf.str();

//   auto k = s.find("\"rad\"");
//   if (k == std::string::npos) return false;
//   k = s.find('[', k);
//   if (k == std::string::npos) return false;
//   auto e = s.find(']', k);
//   if (e == std::string::npos) return false;

//   std::string arr = s.substr(k + 1, e - k - 1);
//   std::array<double,4> v{0,0,0,0};
//   std::stringstream ss(arr);

//   for (int i = 0; i < 4; ++i) {
//     std::string num;
//     if (!std::getline(ss, num, ',')) {
//       if (i == 3) num = arr.substr(arr.find_last_of(',') + 1);
//     }
//     num.erase(0, num.find_first_not_of(" \t\n\r"));
//     num.erase(num.find_last_not_of(" \t\n\r") + 1);
//     v[i] = std::stod(num);
//   }
//   out = v;
//   return true;
// }

// // =========================
// // Anchors (calibration points)
// // =========================
// struct Anchors {
//   // 좌/우는 q11만 사용
//   double left_q11  = +2.8149 + M_PI;
//   double right_q11 = -2.6569 + M_PI;

//   // 상/하는 q12~14 사용
//   double up_q12  = +0.2255, up_q13  = -0.3206, up_q14  = +0.5123;
//   double down_q12= +0.3375, down_q13= +0.7440, down_q14= -0.7624;

//   void load_from_files(const std::string& dir,
//                        const std::string& left_json,
//                        const std::string& right_json,
//                        const std::string& up_json,
//                        const std::string& down_json,
//                        rclcpp::Logger logger)
//   {
//     std::array<double,4> L{left_q11, 0, 0, 0};
//     std::array<double,4> R{right_q11,0, 0, 0};
//     std::array<double,4> U{0, up_q12,  up_q13,  up_q14};
//     std::array<double,4> D{0, down_q12,down_q13,down_q14};
//     std::array<double,4> t;

//     if (load_rad4_from_json(join_paths(dir, left_json), t))  L = t;
//     if (load_rad4_from_json(join_paths(dir, right_json), t)) R = t;
//     if (load_rad4_from_json(join_paths(dir, up_json), t))    U = t;
//     if (load_rad4_from_json(join_paths(dir, down_json), t))  D = t;

//     left_q11  = L[0];
//     right_q11 = R[0];
//     up_q12    = U[1]; up_q13   = U[2]; up_q14   = U[3];
//     down_q12  = D[1]; down_q13 = D[2]; down_q14 = D[3];

//     RCLCPP_INFO(logger,
//       "Anchors:\n LEFT q11=%.4f RIGHT q11=%.4f\n UP q12~14=[%.4f %.4f %.4f]\n DOWN q12~14=[%.4f %.4f %.4f]",
//       left_q11, right_q11, up_q12, up_q13, up_q14, down_q12, down_q13, down_q14);
//   }
// };

// // =========================
// // FusionNode
// // =========================
// class FusionNode : public rclcpp::Node {
// public:
//   FusionNode() : Node("fusion_node") {
//     // ---- Parameters ----
//     declare_parameter<std::string>("anchor_dir",  "/home/b101/chj");
//     declare_parameter<std::string>("left_json",   "left_move.json");
//     declare_parameter<std::string>("right_json",  "right_move.json");
//     declare_parameter<std::string>("up_json",     "up_move.json");
//     declare_parameter<std::string>("down_json",   "down_move.json");

//     declare_parameter<double>("x0_mm",      140.0);
//     declare_parameter<double>("z0_mm",      900.0);
//     declare_parameter<double>("x_span_mm",  180.0);
//     declare_parameter<double>("z_span_mm",  140.0);
//     declare_parameter<double>("alpha",      0.50);

//     // ---- Anchors ----
//     anchors_.load_from_files(
//       get_parameter("anchor_dir").as_string(),
//       get_parameter("left_json").as_string(),
//       get_parameter("right_json").as_string(),
//       get_parameter("up_json").as_string(),
//       get_parameter("down_json").as_string(),
//       get_logger()
//     );

//     // ---- ROS I/F ----
//     // NOTE: depth 0은 유효하지 않음 → 1로 수정
//     pub_ = create_publisher<arm_control_node::msg::CmdPose>("/cmd_pose", rclcpp::QoS(1).transient_local());
//     eye_pose_sub_ = create_subscription<std_msgs::msg::String>(
//       "/eye_pose", 10, std::bind(&FusionNode::on_eye, this, std::placeholders::_1));
//     timer_ = create_wall_timer(20ms, std::bind(&FusionNode::loop, this));

//     RCLCPP_INFO(get_logger(), "✅ fusion_node started (uses JSON anchors, prints sx/sy).");
//   }

// private:
//   // ---- Anchors ----
//   Anchors anchors_;

//   // ---- ROS ----
//   rclcpp::Publisher<arm_control_node::msg::CmdPose>::SharedPtr pub_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eye_pose_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   // ---- Eye state ----
//   std::optional<double> lx_, ly_, lz_, rx_, ry_, rz_;
//   uint64_t ts_{0};
//   bool have_{false};

//   // ---- Smoothing ----
//   std::array<double,4> q_prev_{0,0,0,0};
//   bool first_{true};

//   // --------- Before eye_x,y,z
  

//   // -------------------------
//   // Input handling
//   // -------------------------
//   void on_eye(const std_msgs::msg::String::SharedPtr m) {
//     auto _lx = pick_double(m->data, "lefteye_x");
//     auto _ly = pick_double(m->data, "lefteye_y");
//     auto _lz = pick_double(m->data, "lefteye_z");
//     auto _rx = pick_double(m->data, "righteye_x");
//     auto _ry = pick_double(m->data, "righteye_y");
//     auto _rz = pick_double(m->data, "righteye_z");
//     auto _ts = pick_u64  (m->data, "timestamp");

//     if (_lx && _ly && _lz && _rx && _ry && _rz && _ts) {
//       lx_ = *_lx; ly_ = *_ly; lz_ = *_lz;
//       rx_ = *_rx; ry_ = *_ry; rz_ = *_rz;
//       ts_ = *_ts;
//       have_ = true;
//     }
//   }

//   bool eye_avg(double& x, double& y, double& z) {
//     if (!have_) return false;
//     x = 0.5 * (*lx_) + 0.5 * (*rx_);
//     y = 0.5 * (*ly_) + 0.5 * (*ry_);
//     z = 0.5 * (*lz_) + 0.5 * (*rz_);
//     return true;
//   }

//   // -------------------------
//   // Main loop
//   // -------------------------
//   void loop() {
//     double x_mm, y_mm, z_mm;
//     if (!eye_avg(x_mm, y_mm, z_mm)) return;

//     // Params
//     const double x0 = get_parameter("x0_mm").as_double();
//     const double z0 = get_parameter("z0_mm").as_double();
//     const double xs = std::max(1.0, get_parameter("x_span_mm").as_double());
//     const double zs = std::max(1.0, get_parameter("z_span_mm").as_double());
//     const double alpha = std::clamp(get_parameter("alpha").as_double(), 0.0, 1.0);

//     // Normalized gaze to [-1,1]
//     double sx = std::clamp((x_mm - x0) / xs, -1.0, 1.0);
//     double sy = std::clamp((z0 - z_mm) / zs, -1.0, 1.0);  // ↑위로 갈수록 sy 증가(부호 보정)

//     // L/R, U/D 보간 파라미터
//     double t_lr = 0.5 * (sx + 1.0);
//     double t_ud = 0.5 * (sy + 1.0);

//     // 보간
//     auto lerp = [](double a, double b, double t){ return a * (1.0 - t) + b * t; };

//     double q11 = slerp_angle(anchors_.right_q11,  anchors_.left_q11, t_lr);
//     double q12 = lerp(anchors_.down_q12, anchors_.up_q12, t_ud);
//     double q13 = lerp(anchors_.down_q13, anchors_.up_q13, t_ud);
//     double q14 = lerp(anchors_.down_q14, anchors_.up_q14, t_ud);

//     // 스무딩 (joint11만 wrap 기반, 나머지는 선형)
//     std::array<double,4> q{q11,q12,q13,q14};
//     if (first_) {
//       q_prev_ = q;
//       first_ = false;
//     } else {
//       for (int i = 0; i < 4; ++i) {
//         if (i == 0) {
//           double d = wrap_to_pi(q[i] - q_prev_[i]);
//           q[i] = wrap_to_pi(q_prev_[i] + alpha * d);
//         } else {
//           q[i] = q_prev_[i] * (1.0 - alpha) + q[i] * alpha;
//         }
//       }
//       q_prev_ = q;
//     }

//     // Publish
//     arm_control_node::msg::CmdPose m;
//     m.header.stamp = rclcpp::Time(ts_);  // 그대로 유지
//     m.m11 = q[0]; m.m12 = q[1]; m.m13 = q[2]; m.m14 = q[3];
//     pub_->publish(m);

//     // Log
//     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
//       "Eye(mm)=(%.1f,%.1f,%.1f) sx=%.2f sy=%.2f | q=[%.3f %.3f %.3f %.3f]",
//       x_mm, y_mm, z_mm, sx, sy, q[0], q[1], q[2], q[3]);
//   }
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<FusionNode>());
//   rclcpp::shutdown();
//   return 0;
// }


// fusion_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "arm_control_node/msg/cmd_pose.hpp"

#include <optional>
#include <string>
#include <cmath>
#include <array>
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =========================
// Helpers / Math
// =========================
static inline double wrap_to_pi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double slerp_angle(double a, double b, double t) {
  double da = wrap_to_pi(b - a);
  t = std::clamp(t, 0.0, 1.0);
  return wrap_to_pi(a + da * t);
}

static inline std::string join_paths(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  return (a.back() == '/') ? (a + b) : (a + "/" + b);
}

// =========================
// Tiny JSON helpers (ad-hoc)
// =========================
static std::optional<double> pick_double(const std::string& s, const std::string& key) {
  const std::string tag = "\"" + key + "\":";
  size_t p = s.find(tag);
  if (p == std::string::npos) return std::nullopt;
  p += tag.size();
  size_t e = s.find_first_of(",}", p);
  if (e == std::string::npos) return std::nullopt;
  return std::stod(s.substr(p, e - p));
}

static std::optional<uint64_t> pick_u64(const std::string& s, const std::string& key) {
  const std::string tag = "\"" + key + "\":";
  size_t p = s.find(tag);
  if (p == std::string::npos) return std::nullopt;
  p += tag.size();
  size_t e = s.find_first_of(",}", p);
  if (e == std::string::npos) return std::nullopt;
  return static_cast<uint64_t>(std::stoull(s.substr(p, e - p)));
}

static std::optional<std::string> pick_string(const std::string& s, const std::string& key) {
  const std::string tag = "\"" + key + "\":";
  size_t p = s.find(tag);
  if (p == std::string::npos) return std::nullopt;
  p += tag.size();
  while (p < s.size() && isspace(static_cast<unsigned char>(s[p]))) ++p;
  if (p >= s.size() || s[p] != '"') return std::nullopt;
  size_t q = s.find('"', p + 1);
  if (q == std::string::npos) return std::nullopt;
  return s.substr(p + 1, q - (p + 1));
}

// JSON: {"rad":[r11,r12,r13,r14]} 형태에서 rad 4개를 파싱
static bool load_rad4_from_json(const std::string& path, std::array<double,4>& out) {
  std::ifstream f(path);
  if (!f.is_open()) return false;
  std::stringstream buf; buf << f.rdbuf();
  const std::string s = buf.str();

  auto k = s.find("\"rad\"");
  if (k == std::string::npos) return false;
  k = s.find('[', k);
  if (k == std::string::npos) return false;
  auto e = s.find(']', k);
  if (e == std::string::npos) return false;

  std::string arr = s.substr(k + 1, e - k - 1);
  std::array<double,4> v{0,0,0,0};
  std::stringstream ss(arr);

  for (int i = 0; i < 4; ++i) {
    std::string num;
    if (!std::getline(ss, num, ',')) {
      if (i == 3) num = arr.substr(arr.find_last_of(',') + 1);
    }
    num.erase(0, num.find_first_not_of(" \t\n\r"));
    num.erase(num.find_last_not_of(" \t\n\r") + 1);
    v[i] = std::stod(num);
  }
  out = v;
  return true;
}

// =========================
// Anchors (calibration points)
// =========================
struct Anchors {
  // 좌/우는 q11만 사용
  double left_q11  = +2.8149 + M_PI;
  double right_q11 = -2.6569 + M_PI;

  // 상/하는 q12~14 사용
  double up_q12  = +0.2255, up_q13  = -0.3206, up_q14  = +0.5123;
  double down_q12= +0.3375, down_q13= +0.7440, down_q14= -0.7624;

  void load_from_files(const std::string& dir,
                       const std::string& left_json,
                       const std::string& right_json,
                       const std::string& up_json,
                       const std::string& down_json,
                       rclcpp::Logger logger)
  {
    std::array<double,4> L{left_q11, 0, 0, 0};
    std::array<double,4> R{right_q11,0, 0, 0};
    std::array<double,4> U{0, up_q12,  up_q13,  up_q14};
    std::array<double,4> D{0, down_q12,down_q13,down_q14};
    std::array<double,4> t;

    if (load_rad4_from_json(join_paths(dir, left_json), t))  L = t;
    if (load_rad4_from_json(join_paths(dir, right_json), t)) R = t;
    if (load_rad4_from_json(join_paths(dir, up_json), t))    U = t;
    if (load_rad4_from_json(join_paths(dir, down_json), t))  D = t;

    left_q11  = L[0];
    right_q11 = R[0];
    up_q12    = U[1]; up_q13   = U[2]; up_q14   = U[3];
    down_q12  = D[1]; down_q13 = D[2]; down_q14 = D[3];

    RCLCPP_INFO(logger,
      "Anchors:\n LEFT q11=%.4f RIGHT q11=%.4f\n UP q12~14=[%.4f %.4f %.4f]\n DOWN q12~14=[%.4f %.4f %.4f]",
      left_q11, right_q11, up_q12, up_q13, up_q14, down_q12, down_q13, down_q14);
  }
};

// =========================
// FusionNode
// =========================
class FusionNode : public rclcpp::Node {
public:
  FusionNode() : Node("fusion_node") {
    // ---- Parameters ----
    declare_parameter<std::string>("anchor_dir",  "/home/b101/chj/v1");
    declare_parameter<std::string>("left_json",   "left_move.jsonl");
    declare_parameter<std::string>("right_json",  "right_move.jsonl");
    declare_parameter<std::string>("up_json",     "up_move.jsonl");
    declare_parameter<std::string>("down_json",   "down_move.jsonl");

    declare_parameter<double>("x0_mm",      140.0);
    declare_parameter<double>("z0_mm",      900.0);
    declare_parameter<double>("x_span_mm",  180.0);
    declare_parameter<double>("z_span_mm",  140.0);
    declare_parameter<double>("alpha",      1.00);

    // Manual mode boundaries
    declare_parameter<double>("manual_x_min_mm", -300.0);
    declare_parameter<double>("manual_x_max_mm", 500.0);
    declare_parameter<double>("manual_y_min_mm", -500.0);
    declare_parameter<double>("manual_y_max_mm", 100.0);
    declare_parameter<double>("manual_z_min_mm", 600.0);
    declare_parameter<double>("manual_z_max_mm", 1070.0);
 
    // ---- Anchors ----
    anchors_.load_from_files(
      get_parameter("anchor_dir").as_string(),
      get_parameter("left_json").as_string(),
      get_parameter("right_json").as_string(),
      get_parameter("up_json").as_string(),
      get_parameter("down_json").as_string(),
      get_logger()
    );

    // ---- ROS I/F ----
    pub_ = create_publisher<arm_control_node::msg::CmdPose>("/cmd_pose", rclcpp::QoS(1).transient_local());
    eye_pose_sub_ = create_subscription<std_msgs::msg::String>(
      "/eye_pose", 10, std::bind(&FusionNode::on_eye_pose, this, std::placeholders::_1));
    control_mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/control_mode", 10, std::bind(&FusionNode::on_control_mode, this, std::placeholders::_1));
    manual_pose_sub_ = create_subscription<std_msgs::msg::String>(
      "/manual_pose", 10, std::bind(&FusionNode::on_manual_pose, this, std::placeholders::_1));
    preset_pose_sub_ = create_subscription<std_msgs::msg::String>(
      "/preset_pose", 10, std::bind(&FusionNode::on_preset_pose, this, std::placeholders::_1));

    timer_ = create_wall_timer(20ms, std::bind(&FusionNode::loop, this));

    RCLCPP_INFO(get_logger(), "✅ fusion_node started (uses JSON anchors, prints sx/sy).");
  }

private:
  // ---- control mode ----
  enum class Mode { Auto, Manual, Preset, Fix, Off };
  Mode mode_{Mode::Off};

  // ---- Anchors ----
  Anchors anchors_;

  // ---- ROS ----
  rclcpp::Publisher<arm_control_node::msg::CmdPose>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eye_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr preset_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Eye state (current + previous) ----
  std::optional<double> lx_, ly_, lz_, rx_, ry_, rz_;
  double prev_x_{0.0}, prev_y_{0.0}, prev_z_{0.0};
  uint64_t ts_{0};
  bool have_{false};

  // ---- Manual / Preset inputs ----
  std::optional<double> man_x_, man_y_, man_z_;
    
  std::optional<double> pre_lx_, pre_ly_, pre_lz_, pre_rx_, pre_ry_, pre_rz_;

  // ---- Smoothing ----
  std::array<double,4> q_prev_{0,0,0,0};
  bool first_{true};

  // -------------------------
  // Input handling
  // -------------------------
  // /eye_pose: {lefteye_x, lefteye_y, lefteye_z, righteye_x, righteye_y, righteye_z, timestamp}
  void on_eye_pose(const std_msgs::msg::String::SharedPtr m) {
    auto _lx = pick_double(m->data, "lefteye_x");
    auto _ly = pick_double(m->data, "lefteye_y");
    auto _lz = pick_double(m->data, "lefteye_z");
    auto _rx = pick_double(m->data, "righteye_x");
    auto _ry = pick_double(m->data, "righteye_y");
    auto _rz = pick_double(m->data, "righteye_z");
    auto _ts = pick_u64  (m->data, "timestamp");

    if (_lx && _ly && _lz && _rx && _ry && _rz && _ts) {
      // 현재 갱신
      lx_ = *_lx; ly_ = *_ly; lz_ = *_lz;
      rx_ = *_rx; ry_ = *_ry; rz_ = *_rz;
      ts_ = *_ts;
      have_ = true;
    }
  }

  // /manual_pose: {"x":float,"y":float,"z":float}
  void on_manual_pose(const std_msgs::msg::String::SharedPtr m) {
    auto _x = pick_double(m->data, "x");
    auto _y = pick_double(m->data, "y");
    auto _z = pick_double(m->data, "z");
    if (_x && _y && _z) {
      // 0이면 움직이지 않고, 0이 아니면 -10 또는 +10으로 매핑
      man_x_ = (*_x == 0.0) ? 0.0 : ((*_x > 0.0) ? -10.0 : 10.0);
      man_y_ = (*_y == 0.0) ? 0.0 : ((*_y > 0.0) ? 10.0 : -10.0);
      man_z_ = (*_z == 0.0) ? 0.0 : ((*_z > 0.0) ? -10.0 : 10.0);
      // RCLCPP_INFO(get_logger(), "Manual pose received: x=%.1f y=%.1f z=%.1f", *man_x_, *man_y_, *man_z_);
    } else {
      RCLCPP_WARN(get_logger(), "Manual pose parse failed (expect {\"x\",\"y\",\"z\"}). Raw: %s", m->data.c_str());
    }
  }
  
  // /preset_pose: {"lefteye_x":...,"lefteye_y":...,"lefteye_z":...,"righteye_x":...,"righteye_y":...,"righteye_z":...}
  void on_preset_pose(const std_msgs::msg::String::SharedPtr m) {
    auto _lx = pick_double(m->data, "lefteye_x");
    auto _ly = pick_double(m->data, "lefteye_y");
    auto _lz = pick_double(m->data, "lefteye_z");
    auto _rx = pick_double(m->data, "righteye_x");
    auto _ry = pick_double(m->data, "righteye_y");
    auto _rz = pick_double(m->data, "righteye_z");
    if (_lx && _ly && _lz && _rx && _ry && _rz) {
      pre_lx_ = *_lx; pre_ly_ = *_ly; pre_lz_ = *_lz; 
      pre_rx_ = *_rx; pre_ry_ = *_ry; pre_rz_ = *_rz;
      // 현재 위치 저장
      RCLCPP_INFO(get_logger(), "Preset pose received (L:%.1f,%.1f,%.1f / R:%.1f,%.1f,%.1f)",
                  *pre_lx_, *pre_ly_, *pre_lz_, *pre_rx_, *pre_ry_, *pre_rz_);
    } else {
      RCLCPP_WARN(get_logger(), "Preset pose parse failed (expect 6 eye fields). Raw: %s", m->data.c_str());
    }
  }

  void on_control_mode(const std_msgs::msg::String::SharedPtr m) {
    // /control_mode: {"data":"auto"|"manual"|"preset"|"fix"|"off"}
    Mode new_mode = mode_;
    if (auto s = pick_string(m->data, "data")) {
      if (*s == "auto")   new_mode = Mode::Auto;
      else if (*s == "manual") new_mode = Mode::Manual;
      else if (*s == "preset") new_mode = Mode::Preset;
      else if (*s == "fix")    new_mode = Mode::Fix;
      else if (*s == "off")    new_mode = Mode::Off;
    } else {
      // JSON이 아니라면 전체 문자열로도 허용
      const std::string &v = m->data;
      if (v == "auto")   new_mode = Mode::Auto;
      else if (v == "manual") new_mode = Mode::Manual;
      else if (v == "preset") new_mode = Mode::Preset;
      else if (v == "fix")    new_mode = Mode::Fix;
      else if (v == "off")    new_mode = Mode::Off;
    }
    if (new_mode != mode_) {
      mode_ = new_mode;
      RCLCPP_INFO(get_logger(), "Control mode → %s",
        mode_==Mode::Auto?"auto":mode_==Mode::Manual?"manual":mode_==Mode::Preset?"preset":mode_==Mode::Fix?"fix":"off");
    }
  }

  bool eye_avg(double& x, double& y, double& z) {
    if (!have_) return false;
    x = 0.5 * (*lx_) + 0.5 * (*rx_);
    y = 0.5 * (*ly_) + 0.5 * (*ry_);
    z = 0.5 * (*lz_) + 0.5 * (*rz_);
    prev_x_ = x; prev_y_ = y; prev_z_ = z;
    return true;
  }

  // -------------------------
  // Main loop
  // -------------------------
  void loop() {
    // 위치 소스 선택
    double x_mm=0.0, y_mm=0.0, z_mm=0.0;
    bool ok = false;

    // OFF 모드: 아무 것도 발행하지 않음
    if (mode_ == Mode::Off) {
      x_mm = 178.4;
      y_mm = -227.2;
      z_mm = 769.5;
      ok = true;
      prev_x_ = x_mm; prev_y_ = y_mm; prev_z_ = z_mm;
    };

    // FIX 모드: 마지막 q를 그대로 유지해서 다시 발행
    if (mode_ == Mode::Fix) {
      if (!first_) {
        arm_control_node::msg::CmdPose m;
        m.header.stamp = this->now();
        m.m11 = q_prev_[0]; m.m12 = q_prev_[1]; m.m13 = q_prev_[2]; m.m14 = q_prev_[3];
        pub_->publish(m);
      }
      return;
    }

    rclcpp::Time stamp = this->now();

    // AUTO 모드
    if (mode_ == Mode::Auto) {
      ok = eye_avg(x_mm, y_mm, z_mm);
      if (ok) stamp = rclcpp::Time(ts_);  // auto는 들어온 timestamp 사용

    // MANUAL 모드
    } else if (mode_ == Mode::Manual) {
      x_mm = (have_ ? prev_x_ : 0.0) + *man_x_;
      y_mm = (have_ ? prev_y_ : 0.0) + *man_y_;
      z_mm = (have_ ? prev_z_ : 0.0) + *man_z_; 
      
      // Manual mode 범위 제한
      x_mm = std::clamp(x_mm, get_parameter("manual_x_min_mm").as_double(), get_parameter("manual_x_max_mm").as_double());
      y_mm = std::clamp(y_mm, get_parameter("manual_y_min_mm").as_double(), get_parameter("manual_y_max_mm").as_double());
      z_mm = std::clamp(z_mm, get_parameter("manual_z_min_mm").as_double(), get_parameter("manual_z_max_mm").as_double());

      // stamp = this->now();
      ok = true;

      prev_x_ = x_mm; prev_y_ = y_mm; prev_z_ = z_mm;

    // PRESET 모드
    } else if (mode_ == Mode::Preset) {
        x_mm = 0.5 * (*pre_lx_) + 0.5 * (*pre_rx_);
        y_mm = 0.5 * (*pre_ly_) + 0.5 * (*pre_ry_);
        z_mm = 0.5 * (*pre_lz_) + 0.5 * (*pre_rz_);
        stamp = this->now();  // 프리셋도 현재 시각 사용
        ok = true;

        prev_x_ = x_mm; prev_y_ = y_mm; prev_z_ = z_mm;
    }

    if (!ok) return; // 유효 좌표 없으면 스킵
    
    // Params
    const double x0 = get_parameter("x0_mm").as_double();
    const double z0 = get_parameter("z0_mm").as_double();
    const double xs = std::max(1.0, get_parameter("x_span_mm").as_double());
    const double zs = std::max(1.0, get_parameter("z_span_mm").as_double());
    const double alpha = std::clamp(get_parameter("alpha").as_double(), 0.0, 1.0);

    // Normalized gaze to [-1,1]
    double sx = std::clamp((x_mm - x0) / xs, -1.0, 1.0);
    double sy = std::clamp((z0 - z_mm) / zs, -1.0, 1.0);  // ↑위로 갈수록 sy 증가(부호 보정)

    // L/R, U/D 보간 파라미터
    double t_lr = 0.5 * (sx + 1.0);
    double t_ud = 0.5 * (sy + 1.0);

    // 보간
    auto lerp = [](double a, double b, double t){ return a * (1.0 - t) + b * t; };
    double q11 = slerp_angle(anchors_.right_q11,  anchors_.left_q11, t_lr);
    double q12 = lerp(anchors_.down_q12, anchors_.up_q12, t_ud);
    double q13 = lerp(anchors_.down_q13, anchors_.up_q13, t_ud);
    double q14 = lerp(anchors_.down_q14, anchors_.up_q14, t_ud);

    // 스무딩 (joint11만 wrap 기반, 나머지는 선형)
    std::array<double,4> q{q11,q12,q13,q14};
    if (first_) {
      q_prev_ = q;
      first_ = false;
    } else {
      for (int i = 0; i < 4; ++i) {
        if (i == 0) {
          double d = wrap_to_pi(q[i] - q_prev_[i]);
          q[i] = wrap_to_pi(q_prev_[i] + alpha * d);
        } else {
          q[i] = q_prev_[i] * (1.0 - alpha) + q[i] * alpha;
        }
      }
      q_prev_ = q;
    }

    // Publish
    arm_control_node::msg::CmdPose m;
    m.header.stamp = stamp;
    m.m11 = q[0]; m.m12 = q[1]; m.m13 = q[2]; m.m14 = q[3];
    pub_->publish(m);

    // Log
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
    //   "[%s] Eye(mm)=(%.1f,%.1f,%.1f) sx=%.2f sy=%.2f | q=[%.3f %.3f %.3f %.3f]",
    //   mode_==Mode::Auto?"auto":mode_==Mode::Manual?"manual":mode_==Mode::Preset?"preset":mode_==Mode::Fix?"fix":"off",
    //   x_mm, y_mm, z_mm, sx, sy, q[0], q[1], q[2], q[3]);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
