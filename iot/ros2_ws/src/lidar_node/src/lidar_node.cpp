#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include "rclcpp/qos.hpp"

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)

using std::placeholders::_1;

class PersonDetector : public rclcpp::Node {
public:
  PersonDetector() : Node("lidar_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&PersonDetector::scan_callback, this, _1));

    pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/lidar_dist", 10);

    RCLCPP_INFO(this->get_logger(), "PersonDetector node started (C++ LiDAR only)");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::pair<float, float>> points;

    int count = msg->ranges.size();
    float angle = msg->angle_min;

    for (int i = 0; i < count; ++i, angle += msg->angle_increment) {
      float degree = RAD2DEG(angle);
      if (degree < 0) degree += 360;

      if (degree >= 225.0 && degree <= 315.0) {
        float r = msg->ranges[i];
        if (r >= msg->range_min && r <= msg->range_max) {
          float x = r * cos(angle);
          float y = r * sin(angle);
          points.emplace_back(x, y);
        }
      }
    }

    if (points.empty()) return;

    // 간단한 군집 분리 (인접 거리 기반)
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::vector<std::pair<float, float>> current_cluster;

    const float cluster_dist_thresh = 0.2;

    for (size_t i = 0; i < points.size(); ++i) {
      if (current_cluster.empty()) {
        current_cluster.push_back(points[i]);
      } else {
        float dx = points[i].first - current_cluster.back().first;
        float dy = points[i].second - current_cluster.back().second;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < cluster_dist_thresh) {
          current_cluster.push_back(points[i]);
        } else {
          clusters.push_back(current_cluster);
          current_cluster.clear();
          current_cluster.push_back(points[i]);
        }
      }
    }
    if (!current_cluster.empty())
      clusters.push_back(current_cluster);

    // 사람으로 추정되는 군집 찾기 (폭 0.3 ~ 0.7m)
    float min_dist = std::numeric_limits<float>::max();
    std::pair<float, float> selected_center;

    for (const auto& cluster : clusters) {
      if (cluster.size() < 3) continue;

      float min_x = cluster.front().first;
      float max_x = cluster.front().first;
      float sum_x = 0.0, sum_y = 0.0;

      for (const auto& pt : cluster) {
        if (pt.first < min_x) min_x = pt.first;
        if (pt.first > max_x) max_x = pt.first;
        sum_x += pt.first;
        sum_y += pt.second;
      }

      float width = max_x - min_x;
      if (width >= 0.3 && width <= 0.7) {
        float cx = sum_x / cluster.size();
        float cy = sum_y / cluster.size();
        float dist = std::sqrt(cx * cx + cy * cy);

        if (dist < min_dist) {
          min_dist = dist;
          selected_center = {cx, cy};
        }
      }
    }

    // 검출된 경우만 publish
    if (min_dist < std::numeric_limits<float>::max()) {
      geometry_msgs::msg::PointStamped out;
      out.header = msg->header;
      out.point.x = selected_center.first;
      out.point.y = selected_center.second;
      out.point.z = 0.0;
      pub_->publish(out);
      // RCLCPP_INFO(this->get_logger(), "Detected person at %.2fm", min_dist);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PersonDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
