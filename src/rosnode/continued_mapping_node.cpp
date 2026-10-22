// Humble STB adapter: load in localization, then resume mapping on user request.
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include "nav2_map_server/map_io.hpp"
#include "slam_toolbox/experimental/slam_toolbox_map_and_localization.hpp"
#include "slam_toolbox/serialization.hpp"
#include "std_msgs/msg/string.hpp"

namespace finav
{
class ContinuedMapping : public slam_toolbox::MapAndLocalizationSlamToolbox
{
public:
  explicit ContinuedMapping(const rclcpp::NodeOptions & options)
  : MapAndLocalizationSlamToolbox(options) {}

  void configure() override
  {
    MapAndLocalizationSlamToolbox::configure();
    enterLocalization();
    ssSetLocalizationMode_.reset();
    ssSetLocalizationMode_ = create_service<std_srvs::srv::SetBool>(
      "slam_toolbox/set_localization_mode",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!loaded_) {
          response->success = false;
          response->message = "The saved map has not loaded";
          return;
        }
        if (request->data) {
          save_service_.reset();
          enterLocalization();
        } else if (!mapping_) {
          toggleMode(false);
          // Save directly from the graph; upstream launches a second map subscriber
          // and can time out while this single-threaded service callback is blocked.
          map_saver_.reset();
          save_service_ = create_service<slam_toolbox::srv::SaveMap>("slam_toolbox/save_map",
            [this](const std::shared_ptr<slam_toolbox::srv::SaveMap::Request> req,
            std::shared_ptr<slam_toolbox::srv::SaveMap::Response> resp) {
              resp->result = resp->RESULT_UNDEFINED_FAILURE;
              if (!mapping_ || req->name.data.empty()) {return;}
              nav_msgs::msg::OccupancyGrid map;
              map.header.frame_id = map_frame_;
              map.header.stamp = now();
              map.info.resolution = resolution_;
              map.info.origin.orientation.w = 1.0;
              {
                boost::mutex::scoped_lock lock(smapper_mutex_);
                std::unique_ptr<karto::OccupancyGrid> grid(smapper_->getOccupancyGrid(resolution_));
                if (!grid) {return;}
                vis_utils::toNavMap(grid.get(), map);
              }
              nav2_map_server::SaveParameters params;
              params.map_file_name = req->name.data;
              params.image_format = "pgm";
              params.free_thresh = 0.25;
              params.occupied_thresh = 0.65;
              if (nav2_map_server::saveMapToFile(map, params)) {resp->result = resp->RESULT_SUCCESS;}
            });
          mapping_ = true;
        }
        response->success = true;
        publishState();
      });
    status_ = create_publisher<std_msgs::msg::String>(
      "slam_toolbox/continued_mapping_state", rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {publishState();});
  }

  void loadPoseGraphByParams() override
  {
    LocalizationSlamToolbox::loadPoseGraphByParams();
    if (!loaded_) {
      throw std::runtime_error("Continued mapping requires a valid serialized map");
    }
    // A different graph requires a new session.
    ssDesserialize_.reset();
    publishState();
  }

protected:
  bool serializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> request,
    std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> response) override
  {
    if (!mapping_) {
      response->result = response->RESULT_FAILED_TO_WRITE_FILE;
      return false;
    }
    return SlamToolbox::serializePoseGraphCallback(header, request, response);
  }

  bool deserializePoseGraphCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> request,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response>) override
  {
    if (request->filename.empty() || request->match_type != request->LOCALIZE_AT_POSE) {
      throw std::runtime_error("Continued mapping requires LOCALIZE_AT_POSE and a graph");
    }
    // Upstream returns true even on read failure; verify the read ourselves.
    auto mapper = std::make_unique<karto::Mapper>();
    auto dataset = std::make_unique<karto::Dataset>();
    if (!serialization::read(request->filename, *mapper, *dataset, shared_from_this())) {
      throw std::runtime_error("Cannot load continued mapping graph: " + request->filename);
    }
    loadSerializedPoseGraph(mapper, dataset);
    if (!updateMap()) {throw std::runtime_error("Cannot build grid from the loaded graph");}
    first_measurement_ = true;
    boost::mutex::scoped_lock lock(pose_mutex_);
    processor_type_ = toolbox_types::PROCESS_LOCALIZATION;
    process_near_pose_ = std::make_unique<karto::Pose2>(
      request->initial_pose.x, request->initial_pose.y, request->initial_pose.theta);
    loaded_ = true;
    prefix_ = request->filename;
    return true;
  }

  karto::LocalizedRangeScan * addScan(karto::LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan, karto::Pose2 & pose) override
  {
    auto result = MapAndLocalizationSlamToolbox::addScan(laser, scan, pose);
    if (!mapping_ && seeded_ && result) {
      matched_ = true;
      last_match_ = std::chrono::steady_clock::now();
    }
    return result;
  }

private:
  void enterLocalization()
  {
    toggleMode(true);
    mapping_ = false;
    seeded_ = matched_ = false;
    localization_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        const auto & pose = msg->pose.pose;
        if (msg->header.frame_id != map_frame_ || !std::isfinite(pose.position.x) ||
          !std::isfinite(pose.position.y) || !std::isfinite(pose.orientation.z) ||
          !std::isfinite(pose.orientation.w)) {return;}
        localizePoseCallback(msg);
        seeded_ = true;
        matched_ = false;
        publishState();
      });
  }

  bool ready() const
  {
    return loaded_ && seeded_ && matched_ &&
      std::chrono::steady_clock::now() - last_match_ < std::chrono::seconds(3);
  }

  void publishState()
  {
    if (!status_ || !loaded_) {return;}
    std_msgs::msg::String message;
    message.data = prefix_ + "\n" + (mapping_ ? "mapping" : ready() ? "ready" : "localizing");
    status_->publish(message);
  }

  bool loaded_{false}, mapping_{false}, seeded_{false}, matched_{false};
  std::string prefix_;
  std::chrono::steady_clock::time_point last_match_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<slam_toolbox::srv::SaveMap>::SharedPtr save_service_;
};
}  // namespace finav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<finav::ContinuedMapping> node;
  int result = 0;
  try {
    node = std::make_shared<finav::ContinuedMapping>(rclcpp::NodeOptions());
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
        RCLCPP_WARN(node->get_logger(), "Could not raise pose graph loading stack limit");
      }
    }
    node->configure();
    node->loadPoseGraphByParams();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("slam_toolbox"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  node.reset();
  return result;
}
