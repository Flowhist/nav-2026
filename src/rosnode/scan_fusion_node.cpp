#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class ScanFusionNode : public rclcpp::Node
{
public:
  using LaserScan = sensor_msgs::msg::LaserScan;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan>;

  ScanFusionNode()
  : Node("scan_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    left_sub_(this, "", rmw_qos_profile_sensor_data),
    right_sub_(this, "", rmw_qos_profile_sensor_data)
  {
    declare_parameter<std::string>("left_topic", "/scan_left");
    declare_parameter<std::string>("right_topic", "/scan_right");
    declare_parameter<std::string>("output_topic", "/scan");
    declare_parameter<std::string>("output_frame", "base_link");
    declare_parameter<double>("angle_min", -M_PI);
    declare_parameter<double>("angle_max", M_PI);
    declare_parameter<double>("angle_increment", M_PI / 1800.0);
    declare_parameter<double>("range_min", 0.05);
    declare_parameter<double>("range_max", 25.0);
    declare_parameter<int>("sync_queue_size", 10);
    declare_parameter<double>("input_timeout_sec", 1.0);
    declare_parameter<double>("tf_timeout_sec", 0.1);

    left_topic_ = get_parameter("left_topic").as_string();
    right_topic_ = get_parameter("right_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    angle_min_ = get_parameter("angle_min").as_double();
    angle_max_ = get_parameter("angle_max").as_double();
    angle_increment_ = get_parameter("angle_increment").as_double();
    range_min_ = get_parameter("range_min").as_double();
    range_max_ = get_parameter("range_max").as_double();
    sync_queue_size_ = get_parameter("sync_queue_size").as_int();
    input_timeout_sec_ = get_parameter("input_timeout_sec").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();

    if (angle_increment_ <= 0.0) {
      throw std::runtime_error("angle_increment must be > 0");
    }
    if (angle_max_ <= angle_min_) {
      throw std::runtime_error("angle_max must be > angle_min");
    }

    bin_count_ = static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1U;

    scan_pub_ = create_publisher<LaserScan>(output_topic_, rclcpp::SensorDataQoS());

    left_watch_sub_ = create_subscription<LaserScan>(
      left_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ScanFusionNode::on_left_watch, this, std::placeholders::_1));
    right_watch_sub_ = create_subscription<LaserScan>(
      right_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ScanFusionNode::on_right_watch, this, std::placeholders::_1));

    left_sub_.subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, right_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), left_sub_, right_sub_);
    sync_->registerCallback(
      std::bind(&ScanFusionNode::on_scan_pair, this, std::placeholders::_1, std::placeholders::_2));

    last_left_seen_ = now();
    last_right_seen_ = now();
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&ScanFusionNode::check_inputs, this));

    RCLCPP_INFO(
      get_logger(),
      "scan_fusion_node started | left=%s | right=%s | out=%s | frame=%s | bins=%zu",
      left_topic_.c_str(),
      right_topic_.c_str(),
      output_topic_.c_str(),
      output_frame_.c_str(),
      bin_count_);
  }

private:
  void on_left_watch(const LaserScan::SharedPtr)
  {
    last_left_seen_ = now();
    if (left_stale_) {
      left_stale_ = false;
      RCLCPP_INFO(get_logger(), "Left lidar input recovered: %s", left_topic_.c_str());
    }
  }

  void on_right_watch(const LaserScan::SharedPtr)
  {
    last_right_seen_ = now();
    if (right_stale_) {
      right_stale_ = false;
      RCLCPP_INFO(get_logger(), "Right lidar input recovered: %s", right_topic_.c_str());
    }
  }

  void check_inputs()
  {
    const auto current = now();
    const bool left_timeout =
      (current - last_left_seen_).seconds() > input_timeout_sec_;
    const bool right_timeout =
      (current - last_right_seen_).seconds() > input_timeout_sec_;

    if (left_timeout && !left_stale_) {
      left_stale_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Left lidar input timed out (>%0.2fs): %s",
        input_timeout_sec_,
        left_topic_.c_str());
    }

    if (right_timeout && !right_stale_) {
      right_stale_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Right lidar input timed out (>%0.2fs): %s",
        input_timeout_sec_,
        right_topic_.c_str());
    }
  }

  void on_scan_pair(const LaserScan::ConstSharedPtr left, const LaserScan::ConstSharedPtr right)
  {
    std::vector<float> fused_ranges(bin_count_, std::numeric_limits<float>::infinity());

    if (!accumulate_scan(left, fused_ranges, "left")) {
      return;
    }
    if (!accumulate_scan(right, fused_ranges, "right")) {
      return;
    }

    LaserScan out;
    // Use the later sensor timestamp so SLAM can properly match against
    // historical EKF odom->base_link transforms, avoiding "timestamp earlier
    // than all data in the transform cache" drops.
    out.header.stamp = std::max(
        rclcpp::Time(left->header.stamp),
        rclcpp::Time(right->header.stamp));
    out.header.frame_id = output_frame_;
    out.angle_min = static_cast<float>(angle_min_);
    out.angle_max = static_cast<float>(angle_max_);
    out.angle_increment = static_cast<float>(angle_increment_);
    out.time_increment = 0.0f;
    out.scan_time = std::max(left->scan_time, right->scan_time);
    out.range_min = static_cast<float>(range_min_);
    out.range_max = static_cast<float>(range_max_);
    out.ranges = std::move(fused_ranges);

    scan_pub_->publish(out);
  }

  bool accumulate_scan(
    const LaserScan::ConstSharedPtr & scan,
    std::vector<float> & fused_ranges,
    const char * source_name)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::msg::PointCloud2 transformed;

    try {
      projector_.projectLaser(*scan, cloud, static_cast<double>(scan->range_max));
      const auto transform = tf_buffer_.lookupTransform(
        output_frame_,
        scan->header.frame_id,
        rclcpp::Time(scan->header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      tf2::doTransform(cloud, transformed, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to transform %s scan from %s to %s: %s",
        source_name,
        scan->header.frame_id.c_str(),
        output_frame_.c_str(),
        ex.what());
      return false;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to project %s scan: %s",
        source_name,
        ex.what());
      return false;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const float x = *iter_x;
      const float y = *iter_y;

      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }

      const float range = std::hypot(x, y);
      if (!std::isfinite(range) || range < range_min_ || range > range_max_) {
        continue;
      }

      const double angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_) {
        continue;
      }

      const long index = std::lround((angle - angle_min_) / angle_increment_);
      if (index < 0 || static_cast<std::size_t>(index) >= bin_count_) {
        continue;
      }

      float & current = fused_ranges[static_cast<std::size_t>(index)];
      if (!std::isfinite(current) || range < current) {
        current = range;
      }
    }

    return true;
  }

  std::string left_topic_;
  std::string right_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double angle_min_{};
  double angle_max_{};
  double angle_increment_{};
  double range_min_{};
  double range_max_{};
  double input_timeout_sec_{};
  double tf_timeout_sec_{};
  int sync_queue_size_{};
  std::size_t bin_count_{};
  bool left_stale_{false};
  bool right_stale_{false};

  rclcpp::Time last_left_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_seen_{0, 0, RCL_ROS_TIME};

  laser_geometry::LaserProjection projector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  message_filters::Subscriber<LaserScan> left_sub_;
  message_filters::Subscriber<LaserScan> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<LaserScan>::SharedPtr left_watch_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr right_watch_sub_;
  rclcpp::Publisher<LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
