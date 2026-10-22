// Finav localization adapter for SLAM Toolbox 2.6.x (ROS 2 Humble).
// Initialization follows upstream SlamToolbox::configure; matching stays upstream.
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_map_server/map_io.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "slam_toolbox/serialization.hpp"
#include "slam_toolbox/slam_toolbox_localization.hpp"

namespace finav
{
class FixedLocalization : public slam_toolbox::LocalizationSlamToolbox
{
public:
  explicit FixedLocalization(const rclcpp::NodeOptions & options)
  : LocalizationSlamToolbox(options)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Publish saved YAML/image without rebuilding the grid; restart to change";
    freeze_map_ = declare_parameter("freeze_navigation_map", true, descriptor);
  }

  void configure() override
  {
    if (!freeze_map_) {
      LocalizationSlamToolbox::configure();
      RCLCPP_INFO(get_logger(), "Navigation map updates enabled (upstream STB behavior)");
      return;
    }

    // Reuse upstream setup, omitting its map/graph visualization worker entirely.
    setParams();
    setROSInterfaces();
    setSolver();
    laser_assistant_ = std::make_unique<laser_utils::LaserAssistant>(
      shared_from_this(), tf_.get(), base_frame_);
    pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(
      tf_.get(), base_frame_, odom_frame_);
    scan_holder_ = std::make_unique<laser_utils::ScanHolder>(lasers_);
    closure_assistant_ = std::make_unique<loop_closure_assistant::LoopClosureAssistant>(
      shared_from_this(), smapper_->getMapper(), scan_holder_.get(), state_, processor_type_);
    reprocessing_transform_.setIdentity();
    transform_period_ = declare_parameter("transform_publish_period", 0.05);
    if (!std::isfinite(transform_period_) || transform_period_ < 0.0) {
      throw std::runtime_error("transform_publish_period must be finite and nonnegative");
    }
  }

  void loadPoseGraphByParams() override
  {
    LocalizationSlamToolbox::loadPoseGraphByParams();
    if (freeze_map_) {
      if (!loaded_) {
        throw std::runtime_error("Fixed localization requires a valid map_file_name");
      }
      // This upstream service has an empty response and cannot report rejection.
      // Remove it rather than acknowledge a graph replacement that was not applied.
      ssDesserialize_.reset();
      // Start TF publication only after both map representations loaded successfully.
      threads_.push_back(std::make_unique<boost::thread>(
        [this]() {publishTransformLoop(transform_period_);}));
    }
  }

protected:
  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> request,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> response) override
  {
    if (!freeze_map_) {
      return LocalizationSlamToolbox::deserializePoseGraphCallback(header, request, response);
    }
    // The existing /initialpose and clear_localization_buffer interfaces remain usable.
    // Graph replacement must also replace the fixed grid, so require a service restart.
    if (loaded_) {
      RCLCPP_ERROR(get_logger(), "Fixed map: restart navigation to reload a pose graph");
      return false;
    }
    if (request->filename.empty() || request->match_type != request->LOCALIZE_AT_POSE) {
      throw std::runtime_error("Fixed map requires a pose graph and LOCALIZE_AT_POSE");
    }

    nav_msgs::msg::OccupancyGrid fixed_map;
    const std::string yaml_file = request->filename + ".yaml";
    if (nav2_map_server::loadMapFromYaml(yaml_file, fixed_map) !=
      nav2_map_server::LOAD_MAP_SUCCESS || fixed_map.data.empty() ||
      !std::isfinite(fixed_map.info.resolution) || fixed_map.info.resolution <= 0.0)
    {
      throw std::runtime_error("Cannot load fixed navigation map: " + yaml_file);
    }
    auto mapper = std::make_unique<karto::Mapper>();
    auto dataset = std::make_unique<karto::Dataset>();
    if (!serialization::read(request->filename, *mapper, *dataset, shared_from_this())) {
      throw std::runtime_error("Cannot load localization pose graph: " + request->filename);
    }
    loadSerializedPoseGraph(mapper, dataset);
    // Unlike upstream deserialization, do not call updateMap() here.
    first_measurement_ = true;
    processor_type_ = toolbox_types::PROCESS_LOCALIZATION;
    process_near_pose_ = std::make_unique<karto::Pose2>(
      request->initial_pose.x, request->initial_pose.y, request->initial_pose.theta);
    fixed_map.header.frame_id = map_frame_;
    fixed_map.header.stamp = now();
    map_.map = std::move(fixed_map);
    loaded_ = true;
    sst_->publish(map_.map);
    sstm_->publish(map_.map.info);
    RCLCPP_INFO(get_logger(), "Fixed navigation map loaded: %s (%u x %u); grid updates disabled",
      yaml_file.c_str(), map_.map.info.width, map_.map.info.height);
    return true;
  }

private:
  bool freeze_map_{true};
  bool loaded_{false};
  double transform_period_{0.05};
};
}  // namespace finav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<finav::FixedLocalization> node;
  int result = 0;
  try {
    node = std::make_shared<finav::FixedLocalization>(rclcpp::NodeOptions());
    const auto stack_size = node->declare_parameter<int64_t>("stack_size_to_use", 40000000);
    struct rlimit limit;
    if (stack_size > 0 && getrlimit(RLIMIT_STACK, &limit) == 0 &&
      limit.rlim_cur < static_cast<rlim_t>(stack_size))
    {
      limit.rlim_cur = static_cast<rlim_t>(stack_size);
      if (limit.rlim_max != RLIM_INFINITY && limit.rlim_cur > limit.rlim_max) {
        limit.rlim_cur = limit.rlim_max;
      }
      if (setrlimit(RLIMIT_STACK, &limit) != 0) {
        RCLCPP_WARN(node->get_logger(), "Could not raise stack limit for pose graph loading");
      }
    }
    node->configure();
    node->loadPoseGraphByParams();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("slam_toolbox"), "%s", error.what());
    result = 1;
  }
  // Upstream worker threads exit on context shutdown; shut down before destroying node.
  rclcpp::shutdown();
  node.reset();
  return result;
}
