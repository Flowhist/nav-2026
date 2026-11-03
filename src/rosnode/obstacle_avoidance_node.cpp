#include "finav/avoidance/core.hpp"
#include "finav/msg/avoidance_request.hpp"
#include "finav/msg/avoidance_result.hpp"
#include "finav/msg/fused_scan.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>
#include <fstream>
#include <sys/resource.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <interfaces/msg/ks114_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/empty.hpp>

namespace fa = finav::avoidance;
using Steady = std::chrono::steady_clock;
static double seconds() { return std::chrono::duration<double>(Steady::now().time_since_epoch()).count(); }
static double stamp(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec*1e-9; }

class AvoidanceNode : public rclcpp::Node {
 public:
  AvoidanceNode() : Node("obstacle_avoidance") {
    fa::Config c;
    c.resolution = declare_parameter("resolution",c.resolution);
    c.range = declare_parameter("range",c.range);
    c.memory_seconds = declare_parameter("memory_seconds",c.memory_seconds);
    c.margin = declare_parameter("margin",c.margin);
    c.min_height = declare_parameter("min_height",c.min_height);
    c.depth_margin = declare_parameter("depth_margin",c.depth_margin);
    c.max_linear = declare_parameter("max_linear",c.max_linear);
    c.max_angular = declare_parameter("max_angular",c.max_angular);
    c.reaction_seconds = declare_parameter("reaction_seconds",c.reaction_seconds);
    c.linear_deceleration = declare_parameter("linear_deceleration",c.linear_deceleration);
    c.angular_deceleration = declare_parameter("angular_deceleration",c.angular_deceleration);
    c.paths = declare_parameter("paths",c.paths);
    c.intent_lookahead = declare_parameter("intent_lookahead",c.intent_lookahead);
    const auto footprint = declare_parameter<std::vector<double>>("footprint",std::vector<double>{});
    ultrasonic_mounts_=declare_parameter<std::vector<double>>("ultrasonic_mounts",std::vector<double>{});
    if (!ultrasonic_mounts_.empty() && ((ultrasonic_mounts_.size()!=16 && ultrasonic_mounts_.size()!=24) ||
        !std::all_of(ultrasonic_mounts_.begin(),ultrasonic_mounts_.end(),
                     [](double v) { return std::isfinite(v); })))
      throw std::invalid_argument("Invalid ultrasonic mounts");
    if (footprint.size()!=5) throw std::invalid_argument("Use launch to load robot geometry");
    core_ = std::make_unique<fa::Core>(c,fa::Footprint{footprint[0],footprint[1],footprint[2],footprint[3],footprint[4]});
    sensor_timeout_ = declare_parameter("sensor_timeout",.5);
    goal_timeout_ = declare_parameter("goal_timeout",.5);
    ultrasonic_stop_ = declare_parameter("ultrasonic_stop",.5);
    bus_ = declare_parameter<std::string>("ultrasonic_bus","bus_0");
    for (double t : {sensor_timeout_,goal_timeout_})
      if (!std::isfinite(t) || t <= 0 || t > .5) throw std::invalid_argument("Timeout must be in (0, 0.5]");
    if (!std::isfinite(ultrasonic_stop_) || ultrasonic_stop_ <= 0)
      throw std::invalid_argument("Ultrasonic stop distance must be finite and positive");
    ultrasonic_limits_=declare_parameter<std::vector<double>>(
        "ultrasonic_stop_distances",std::vector<double>{});
    if (ultrasonic_limits_.empty()) ultrasonic_limits_.assign(4,ultrasonic_stop_);
    if ((ultrasonic_limits_.size()!=4 && ultrasonic_limits_.size()!=6) ||
        !std::all_of(ultrasonic_limits_.begin(),ultrasonic_limits_.end(),
        [](double v) { return std::isfinite(v) && v>0; }))
      throw std::invalid_argument("ultrasonic_stop_distances must contain four or six finite positive distances (0x10 onward)");
    inputs_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    worker_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options; options.callback_group=inputs_group_;
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    scan_sub_ = create_subscription<finav::msg::FusedScan>("/avoidance/scan",qos,
      [this](finav::msg::FusedScan::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double wall=now().seconds(),lt=stamp(m->left.header.stamp),rt=stamp(m->right.header.stamp);
        if (lt<=0 || rt<=0 || lt>wall || rt>wall || wall-lt>=sensor_timeout_ || wall-rt>=sensor_timeout_) {
          input_.scan.reset(); input_.scans.clear(); return;
        }
        if (input_.scan && (stamp(m->left.header.stamp)<=stamp(input_.scan->left.header.stamp) ||
            stamp(m->right.header.stamp)<=stamp(input_.scan->right.header.stamp))) return;
        input_.scan=m; input_.scan_received=seconds();
        input_.scans.push_back({m,input_.scan_received});
        if (input_.scans.size()>8) input_.scans.pop_front();
      },options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic",std::string("/odom_encoder")),qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        fa::Pose before,after;
        const double dt=input_.odom ? stamp(m->header.stamp)-stamp(input_.odom->header.stamp) : 0;
        if (input_.odom && dt==0) return;
        if (input_.odom && (dt<0 || !odom_pose(*input_.odom,before) || !odom_pose(*m,after) ||
            std::hypot(after.x-before.x,after.y-before.y)>.1+.6*dt ||
            std::abs(std::remainder(after.yaw-before.yaw,2*M_PI))>.1+.5*dt)) {
          input_.odoms.clear(); input_.scans.clear(); ++input_.odom_epoch;
        }
        input_.odom=m; input_.odom_received=seconds();
        input_.odoms.push_back({m,input_.odom_received});
        if (input_.odoms.size()>32) input_.odoms.pop_front();
      },options);
    ultrasonic_sub_ = create_subscription<interfaces::msg::KS114State>(
      declare_parameter("ultrasonic_topic",std::string("/ultrasonic/device/state")),qos,
      [this](interfaces::msg::KS114State::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_); input_.ultrasonic=m; input_.ultrasonic_received=seconds();
      },options);
    request_sub_ = create_subscription<finav::msg::AvoidanceRequest>("/avoidance/request",rclcpp::QoS(1),
      [this](finav::msg::AvoidanceRequest::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double timestamp=stamp(m->header.stamp),wall=now().seconds();
        if (timestamp<=input_.request_barrier) return;
        if (input_.request && m->session==input_.request->session && m->sequence<=input_.request->sequence) return;
        if (timestamp<=wall && wall-timestamp<=.1) input_.request_barrier=timestamp;
        input_.request=m; input_.request_received=seconds();
      },options);
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/avoidance/goal",rclcpp::QoS(1),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double timestamp=stamp(m->header.stamp),wall=now().seconds();
        if (timestamp<=input_.goal_barrier || timestamp>wall || wall-timestamp>=goal_timeout_ ||
            m->header.frame_id!="odom" || !std::isfinite(m->point.x) || !std::isfinite(m->point.y)) return;
        input_.goal=m; input_.goal_received=seconds(); input_.goal_barrier=stamp(m->header.stamp);
      },options);
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>("/avoidance/cancel",rclcpp::QoS(1),
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        input_.goal.reset(); input_.goal_barrier=now().seconds();
      },options);
    result_pub_ = create_publisher<finav::msg::AvoidanceResult>("/avoidance/result",1);
    suggestion_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/avoidance/suggestion",1);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/avoidance/status",1);
    scene_pub_ = create_publisher<sensor_msgs::msg::PointCloud>("/avoidance/obstacles",rclcpp::SensorDataQoS().keep_last(1));
    // Report the startup configuration actually used by the core.
    for (const auto& key : {"resolution","range","memory_seconds","margin","min_height","depth_margin",
        "max_linear","max_angular","reaction_seconds","linear_deceleration","angular_deceleration",
        "sensor_timeout","goal_timeout","ultrasonic_stop","paths","intent_lookahead","odom_topic","ultrasonic_topic"})
      config_values_.emplace_back(key,get_parameter(key).value_to_string());
    for (const auto& key : {"footprint","ultrasonic_mounts"}) {
      const auto values=get_parameter(key).as_double_array();
      for (size_t i=0;i<values.size();++i)
        config_values_.emplace_back(std::string(key)+"_"+std::to_string(i),std::to_string(values[i]));
    }
    for (size_t i=0;i<ultrasonic_limits_.size();++i)
      config_values_.emplace_back("us_"+std::to_string(0x10+i)+"_stop_m",std::to_string(ultrasonic_limits_[i]));
    timer_ = create_wall_timer(std::chrono::milliseconds(50),[this] { tick(); },worker_group_);
    RCLCPP_INFO(get_logger(),"Intent correction worker: router owns chassis output; MRPT 2.4.2, 2 executor threads");
  }

 private:
  template<class Message> struct Sample {
    typename Message::ConstSharedPtr message;
    double received;
  };
  struct Input {
    finav::msg::FusedScan::ConstSharedPtr scan;
    nav_msgs::msg::Odometry::ConstSharedPtr odom;
    interfaces::msg::KS114State::ConstSharedPtr ultrasonic;
    finav::msg::AvoidanceRequest::ConstSharedPtr request;
    geometry_msgs::msg::PointStamped::ConstSharedPtr goal;
    std::deque<Sample<finav::msg::FusedScan>> scans;
    std::deque<Sample<nav_msgs::msg::Odometry>> odoms;
    uint64_t odom_epoch=0;
    double scan_received=0,odom_received=0,ultrasonic_received=0,request_received=0,goal_received=0,goal_barrier=0,request_barrier=0;
  } input_;
  std::mutex mutex_;
  std::unique_ptr<fa::Core> core_;
  fa::Pose previous_pose_;
  fa::Pose captured_pose_;
  fa::Pose captured_right_pose_;
  uint64_t odom_epoch_=0;
  double sync_before_=0,sync_after_=0;
  bool sync_matched_=false;
  double previous_odom_stamp_=0,scan_stamp_=0,last_status_=0,last_plan_=0,last_clock_=0;
  double sensor_timeout_,goal_timeout_,ultrasonic_stop_;
  double sensor_deadline_=0;
  std::string bus_;
  std::vector<double> ultrasonic_mounts_,ultrasonic_limits_;
  std::vector<fa::Point> near_echoes_;
  bool near_without_mount_=false;
  std::vector<std::pair<std::string,std::string>> config_values_;
  uint64_t frames_=0,plans_=0,request_reviews_=0,corrections_=0;
  uint64_t status_frames_=0,status_plans_=0,ticks_=0,deadline_misses_=0;
  double tick_sum_=0,tick_max_=0,last_cpu_=0,started_=seconds();
  bool scan_valid_=false;
  rclcpp::CallbackGroup::SharedPtr inputs_group_,worker_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<finav::msg::FusedScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<interfaces::msg::KS114State>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<finav::msg::AvoidanceRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Publisher<finav::msg::AvoidanceResult>::SharedPtr result_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr suggestion_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr scene_pub_;
  double last_scene_=0;

  bool fresh(double timestamp,double received,double wall,double mono,double limit) const {
    return timestamp>0 && wall>=timestamp && wall-timestamp<limit && mono>=received && mono-received<limit;
  }
  static bool odom_pose(const nav_msgs::msg::Odometry& odom,fa::Pose& pose) {
    if (odom.header.frame_id!="odom" || odom.child_frame_id!="base_link") return false;
    const auto& q=odom.pose.pose.orientation;
    const auto& p=odom.pose.pose.position;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm) || std::abs(norm-1)>.01 || std::abs(q.x)+std::abs(q.y)>.05 ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x)+std::abs(p.y)>1e6 ||
        !std::isfinite(odom.twist.twist.linear.x) || !std::isfinite(odom.twist.twist.angular.z)) return false;
    pose={p.x,p.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
    return true;
  }

  bool match_scan(Input& in,double wall,double mono) {
    // Search newest first; wait for bracketing odometry without extrapolating a pose.
    auto match=[&](double t,fa::Pose& result) {
      const Sample<nav_msgs::msg::Odometry>* lower=nullptr;
      for (const auto& upper:in.odoms) {
        const double b=stamp(upper.message->header.stamp);
        if (b<=t) lower=&upper;
        if (b<t || !lower) continue;
        const double a=stamp(lower->message->header.stamp);
        if (t-a>.1 || b-t>.1 ||
            !fresh(a,lower->received,wall,mono,sensor_timeout_) ||
            !fresh(b,upper.received,wall,mono,sensor_timeout_)) break;
        fa::Pose first,last;
        if (!odom_pose(*lower->message,first) || !odom_pose(*upper.message,last)) break;
        const double fraction=b>a ? (t-a)/(b-a) : 0;
        result={first.x+fraction*(last.x-first.x),first.y+fraction*(last.y-first.y),
            first.yaw+fraction*std::remainder(last.yaw-first.yaw,2*M_PI)};
        sync_before_=std::max(sync_before_,t-a); sync_after_=std::max(sync_after_,b-t);
        sensor_deadline_=std::min({sensor_deadline_,lower->received+sensor_timeout_,
            upper.received+sensor_timeout_,mono-(wall-a)+sensor_timeout_});
        return true;
      }
      return false;
    };
    for (auto scan=in.scans.rbegin();scan!=in.scans.rend();++scan) {
      const double lt=stamp(scan->message->left.header.stamp),rt=stamp(scan->message->right.header.stamp);
      if (std::min(lt,rt)<scan_stamp_ || !fresh(lt,scan->received,wall,mono,sensor_timeout_) ||
          !fresh(rt,scan->received,wall,mono,sensor_timeout_)) continue;
      if (!match(lt,captured_pose_) || !match(rt,captured_right_pose_)) continue;
      in.scan=scan->message; in.scan_received=scan->received;
      sync_matched_=true;
      return true;
    }
    return false;
  }

  std::string sensors(Input& in,double wall,double mono,fa::Pose& pose,fa::Velocity& actual) {
    sync_before_=sync_after_=0;
    sync_matched_=false;
    if (in.odom_epoch!=odom_epoch_) {
      odom_epoch_=in.odom_epoch; core_->clear(); scan_stamp_=0; previous_odom_stamp_=0;
      return "odom_discontinuity";
    }
    if (!in.odom || !fresh(stamp(in.odom->header.stamp),in.odom_received,wall,mono,sensor_timeout_)) return "odom_expired";
    if (in.odom->header.frame_id!="odom" || in.odom->child_frame_id!="base_link") return "odom_frame";
    if (!odom_pose(*in.odom,pose)) return "odom_invalid";
    actual={in.odom->twist.twist.linear.x,in.odom->twist.twist.angular.z};
    if (!std::isfinite(actual.linear) || !std::isfinite(actual.angular)) return "odom_invalid";
    const double odom_stamp=stamp(in.odom->header.stamp), dt=odom_stamp-previous_odom_stamp_;
    if (previous_odom_stamp_>0 && (dt<0 || (dt>0 &&
        (std::hypot(pose.x-previous_pose_.x,pose.y-previous_pose_.y)>.1+.6*dt ||
         std::abs(std::remainder(pose.yaw-previous_pose_.yaw,2*M_PI))>.1+.5*dt)))) {
      core_->clear(); previous_odom_stamp_=odom_stamp; previous_pose_=pose; return "odom_discontinuity";
    }
    previous_odom_stamp_=odom_stamp; previous_pose_=pose;
    if (!in.scan || !fresh(stamp(in.scan->left.header.stamp),in.scan_received,wall,mono,sensor_timeout_) ||
        !fresh(stamp(in.scan->right.header.stamp),in.scan_received,wall,mono,sensor_timeout_)) return "lidar_expired";
    auto valid=[](const sensor_msgs::msg::PointCloud& cloud,const geometry_msgs::msg::Point32& origin) {
      if (cloud.header.frame_id!="base_link" || cloud.points.empty() || cloud.points.size()>10000 ||
          !std::isfinite(origin.x) || !std::isfinite(origin.y) || std::hypot(origin.x,origin.y)>2) return false;
      for (const auto& p:cloud.points)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::hypot(p.x,p.y)>100) return false;
      return true;
    };
    if (!valid(in.scan->left,in.scan->left_origin) || !valid(in.scan->right,in.scan->right_origin) ||
        std::abs(stamp(in.scan->left.header.stamp)-stamp(in.scan->right.header.stamp))>.1) return "lidar_invalid";
    sensor_deadline_=std::min({in.scan_received,in.odom_received,
        mono-(wall-std::min(stamp(in.scan->left.header.stamp),stamp(in.scan->right.header.stamp))),
        mono-(wall-odom_stamp)})+sensor_timeout_;
    if (!match_scan(in,wall,mono)) return "lidar_odom_unsynchronized";
    if (!valid(in.scan->left,in.scan->left_origin) || !valid(in.scan->right,in.scan->right_origin) ||
        std::abs(stamp(in.scan->left.header.stamp)-stamp(in.scan->right.header.stamp))>.1) return "lidar_invalid";
    sensor_deadline_=std::min({sensor_deadline_,in.scan_received+sensor_timeout_,
        mono-(wall-std::min(stamp(in.scan->left.header.stamp),stamp(in.scan->right.header.stamp)))+sensor_timeout_});
    return {};
  }

  std::string ultrasonic(const Input& in,double wall,double mono) {
    near_echoes_.clear(); near_without_mount_=false;
    if (!in.ultrasonic || mono-in.ultrasonic_received>=sensor_timeout_) return "ultrasonic_expired";
    for (int address=0x10; address<0x10+static_cast<int>(ultrasonic_limits_.size()); ++address) {
      int count=0;
      for (const auto& r:in.ultrasonic->sensors) if (r.bus==bus_ && r.address==address) {
        ++count;
        if (!r.online || !r.valid || r.consecutive_failures || !std::isfinite(r.distance_m) || r.distance_m<=0)
          return "ultrasonic_invalid";
        if (!fresh(stamp(r.stamp),in.ultrasonic_received,wall,mono,sensor_timeout_)) return "ultrasonic_sample_expired";
        sensor_deadline_=std::min(sensor_deadline_,std::min(in.ultrasonic_received,
            mono-(wall-stamp(r.stamp)))+sensor_timeout_);
        if (r.distance_m<=ultrasonic_limits_.at(address-0x10)) {
          if (ultrasonic_mounts_.size()<static_cast<size_t>((address-0x10+1)*4)) near_without_mount_=true;
          else {
            const int i=(address-0x10)*4;
            near_echoes_.push_back({ultrasonic_mounts_[i]+r.distance_m*std::cos(ultrasonic_mounts_[i+3]),
                                   ultrasonic_mounts_[i+1]+r.distance_m*std::sin(ultrasonic_mounts_[i+3])});
          }
        }
      }
      if (count!=1) return "ultrasonic_missing_or_duplicate";
    }
    return {};
  }

  bool ultrasonic_blocks(fa::Velocity velocity) const {
    if (near_without_mount_) return true;
    for (const auto& echo : near_echoes_)
      if (core_->approaches(echo,velocity)) return true;
    return false;
  }

  bool observe(const Input& in,const fa::Pose& pose,double wall,double mono) {
    const auto& scan=*in.scan;
    const double t=std::min(stamp(scan.left.header.stamp),stamp(scan.right.header.stamp));
    if (t<scan_stamp_) { core_->clear(); return false; }
    if (t==scan_stamp_) return scan_valid_;
    std::vector<fa::Ray> rays;
    auto append=[&](const sensor_msgs::msg::PointCloud& cloud,const geometry_msgs::msg::Point32& origin,
                    const fa::Pose& captured) {
      const double c=std::cos(captured.yaw),s=std::sin(captured.yaw);
      auto world=[&](double x,double y) { return fa::Point{captured.x+c*x-s*y,captured.y+s*x+c*y}; };
      const auto start=world(origin.x,origin.y);
      for (const auto& p:cloud.points) rays.push_back({start,world(p.x,p.y)});
    };
    append(scan.left,scan.left_origin,captured_pose_);
    append(scan.right,scan.right_origin,captured_right_pose_);
    scan_valid_=core_->observe_scan(rays,pose,mono-(wall-t));
    scan_stamp_=t; ++frames_;
    return scan_valid_;
  }

  void tick() {
    const double began=seconds(),wall=now().seconds();
    Input in; { std::lock_guard<std::mutex> lock(mutex_); in=input_; }
    fa::Pose pose; fa::Velocity actual;
    bool pose_valid=false;
    std::string reason;
    fa::Decision decision{{},"idle"};
    fa::PlanningTrace planning_trace;
    bool planning_evaluated=false;
    fa::Velocity suggestion;
    try {
      if (last_clock_>wall) { core_->clear(); scan_stamp_=0; previous_odom_stamp_=0; reason="clock_reset"; }
      last_clock_=wall;
      if (reason.empty()) reason=sensors(in,wall,began,pose,actual);
      pose_valid=reason.empty() || reason=="lidar_expired" || reason=="lidar_odom_unsynchronized";
      if (reason.empty() && !observe(in,pose,wall,began)) reason="lidar_invalid";
      if (reason.empty()) reason=ultrasonic(in,wall,began);
      const bool manual=in.request && (in.request->source=="joystick" ||
          in.request->source=="web" || in.request->source=="keyboard");
      const bool near=near_without_mount_ || !near_echoes_.empty();
      if (reason.empty() && near && !manual) reason="ultrasonic_blocked";
      if (count_publishers("/avoidance/scan")>1 || count_publishers("/avoidance/request")>1 ||
          count_publishers("/avoidance/goal")>1) reason="input_publisher_conflict";
      if (in.request) {
        const auto& r=*in.request;
        const double lifetime=r.valid_for.sec+r.valid_for.nanosec*1e-9;
        decision.reason=reason;
        if (decision.reason.empty()) {
          if (r.header.frame_id!="base_link" || r.session.empty() || r.session.size()>128 || r.source.empty() ||
              r.source.size()>128 || lifetime<=0 || lifetime>.1 ||
              !fresh(stamp(r.header.stamp),in.request_received,now().seconds(),seconds(),lifetime))
            decision.reason="request_invalid_or_expired";
          else if (r.velocity.linear.y!=0 || r.velocity.linear.z!=0 || r.velocity.angular.x!=0 || r.velocity.angular.y!=0)
            decision.reason="unsupported_velocity";
          else {
            planning_evaluated=true;
            decision=core_->correct({r.velocity.linear.x,r.velocity.angular.z},actual,pose,seconds(),manual,&planning_trace);
            // Recheck the actual output too: a detour may approach an echo the original intent avoided.
            if (manual && (ultrasonic_blocks(actual) || ultrasonic_blocks(decision.velocity)))
              decision={{},"ultrasonic_blocked"};
            ++request_reviews_;
            if (decision.reason=="corrected") ++corrections_;
          }
        }
      }
      if (reason.empty() && !near && in.goal && began-last_plan_>=.1 && frames_!=last_planned_frame_ &&
          fresh(stamp(in.goal->header.stamp),in.goal_received,wall,began,goal_timeout_)) {
        const auto& goal=*in.goal;
        if (goal.header.frame_id=="odom" && std::isfinite(goal.point.x) && std::isfinite(goal.point.y) &&
            std::hypot(goal.point.x-pose.x,goal.point.y-pose.y)<=3) {
          const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
          const double x=goal.point.x-pose.x,y=goal.point.y-pose.y;
          suggestion=core_->plan({c*x+s*y,-s*x+c*y},pose,seconds());
          const auto checked=core_->check(suggestion,actual,pose,seconds());
          suggestion=checked.velocity; goal_reason_=checked.reason;
          last_suggestion_=suggestion; last_goal_=in.goal; proposal_stamp_=now();
          ++plans_; last_plan_=began; last_planned_frame_=frames_;
        } else goal_reason_="goal_invalid";
      } else {
        goal_reason_=reason.empty()?(near?"ultrasonic_blocked":"idle_or_waiting_for_new_frame"):reason;
        if (reason.empty() && !near && in.goal && in.goal==last_goal_ && began-last_plan_<.15)
          suggestion=core_->check(last_suggestion_,actual,pose,seconds()).velocity;
      }
    } catch (const std::exception& e) {
      reason="processing_error"; decision={{},reason}; suggestion={}; core_->clear();
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Avoidance processing: %s",e.what());
    }
    const double elapsed=seconds()-began;
    ++ticks_; tick_sum_+=elapsed; tick_max_=std::max(tick_max_,elapsed);
    if (elapsed>.05) ++deadline_misses_;
    if (reason.empty() && seconds()>=sensor_deadline_) {
      decision={{},"sensor_expired_during_compute"}; suggestion={}; reason="sensor_expired_during_compute";
    }
    // A late computation must never turn into a newly valid proposal.
    if (elapsed>.05) { decision={{},"compute_deadline"}; suggestion={}; reason="compute_deadline"; }
    if (in.request) {
      const auto& r=*in.request;
      const double lifetime=r.valid_for.sec+r.valid_for.nanosec*1e-9;
      if (!fresh(stamp(r.header.stamp),in.request_received,now().seconds(),seconds(),lifetime))
        decision={{},"request_expired"};
      std::lock_guard<std::mutex> lock(mutex_);
      if (input_.request!=in.request && (!input_.request ||
          input_.request->session!=r.session || input_.request->source!=r.source ||
          input_.request->velocity.linear.x!=r.velocity.linear.x ||
          input_.request->velocity.angular.z!=r.velocity.angular.z)) decision={{},"request_superseded"};
      finav::msg::AvoidanceResult result;
      result.header=r.header; result.session=r.session; result.sequence=r.sequence; result.source=r.source;
      result.valid_for=r.valid_for; result.reason=decision.reason;
      result.velocity.linear.x=decision.velocity.linear; result.velocity.angular.z=decision.velocity.angular;
      result_pub_->publish(result);
    }
    geometry_msgs::msg::TwistStamped proposal;
    proposal.header.frame_id="base_link";
    proposal.header.stamp=(suggestion.linear!=0 || suggestion.angular!=0)?proposal_stamp_:now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (input_.goal!=in.goal || !in.goal ||
          !fresh(stamp(in.goal->header.stamp),in.goal_received,now().seconds(),seconds(),goal_timeout_)) suggestion={};
    }
    proposal.twist.linear.x=suggestion.linear; proposal.twist.angular.z=suggestion.angular;
    suggestion_pub_->publish(proposal);
    if (began-last_scene_>=.2 && scene_pub_->get_subscription_count()>0) {
      sensor_msgs::msg::PointCloud cloud;
      cloud.header.frame_id="base_link"; cloud.header.stamp=now();
      // Reuse the grid; expire cells before projecting into the current robot frame.
      if (pose_valid && in.odom && scan_valid_ &&
          fresh(stamp(in.odom->header.stamp),in.odom_received,wall,began,sensor_timeout_)) {
        const auto& points=core_->obstacles();
        const size_t stride=std::max<size_t>(1,(points.size()+4095)/4096);
        const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
        for (size_t i=0;i<points.size();i+=stride) {
          const auto& p=points[i];
          if (core_->state(p.x,p.y,began)!=fa::CellState::Occupied) continue;
          geometry_msgs::msg::Point32 point;
          const double x=p.x-pose.x,y=p.y-pose.y;
          point.x=c*x+s*y; point.y=-s*x+c*y;
          cloud.points.push_back(point);
        }
      }
      scene_pub_->publish(cloud); last_scene_=began;
    }
    if (began-last_status_>=.2) {
      diagnostic_msgs::msg::DiagnosticArray status;
      status.header.stamp=now(); status.status.resize(1);
      auto& item=status.status[0]; item.name="avoidance_observation"; item.hardware_id="finav";
      item.level=reason.empty()?0:1; item.message=reason.empty()?"observation_only":reason;
      auto add=[&](const std::string& key,const std::string& value) {
        diagnostic_msgs::msg::KeyValue pair; pair.key=key; pair.value=value; item.values.push_back(pair);
      };
      add("request",decision.reason); add("goal",goal_reason_); add("frames",std::to_string(frames_));
      add("plans",std::to_string(plans_)); add("obstacles",std::to_string(core_->obstacle_count()));
      add("request_reviews",std::to_string(request_reviews_)); add("corrections",std::to_string(corrections_));
      add("tick_ms",std::to_string(elapsed*1000)); add("drives_chassis","false");
      auto number=[&](const std::string& key,double value) {
        if (std::isfinite(value)) add(key,std::to_string(value));
      };
      add("planning_evaluated",planning_evaluated?"true":"false");
      add("observation_source","dual_lidar");
      add("lidar_odom_matched",sync_matched_?"true":"false");
      number("lidar_odom_before_s",sync_before_);
      number("lidar_odom_after_s",sync_after_);
      if (sync_matched_) {
        number("lidar_pose_x",captured_pose_.x); number("lidar_pose_y",captured_pose_.y);
        number("lidar_pose_yaw",captured_pose_.yaw);
      }
      add("planning_original_stage",planning_trace.original_stage);
      add("planning_original_reason",planning_trace.original_reason);
      add("planning_search_status",planning_trace.search_status);
      number("planning_attempts",planning_trace.attempts);
      number("planning_slowdown_attempts",planning_trace.slowdown_attempts);
      number("planning_slowdown_scale",planning_trace.slowdown_scale);
      for (int i=0;i<planning_trace.slowdown_attempts;++i)
        add("planning_slowdown_"+std::to_string(i)+"_reason",planning_trace.slowdown_reasons.at(i));
      for (int i=0;i<planning_trace.attempts;++i) {
        const auto& candidate=planning_trace.candidates.at(i);
        const std::string prefix="planning_candidate_"+std::to_string(i)+"_";
        add(prefix+"stage",candidate.stage); add(prefix+"reason",candidate.reason);
        number(prefix+"planner_speed",candidate.planner_speed);
        number(prefix+"planner_direction",candidate.planner_direction);
        number(prefix+"linear",candidate.velocity.linear);
        number(prefix+"angular",candidate.velocity.angular);
      }
      auto age=[&](const std::string& key,double timestamp) { number(key,wall-timestamp); };
      const double interval=began-(last_status_>0?last_status_:started_);
      add("schema","2"); number("uptime_s",began-started_);
      number("tick_mean_ms",1000*tick_sum_/ticks_); number("tick_max_ms",1000*tick_max_);
      number("deadline_misses",deadline_misses_);
      if (interval>0) {
        number("frames_hz",(frames_-status_frames_)/interval);
        number("plans_hz",(plans_-status_plans_)/interval);
        struct rusage usage{};
        if (getrusage(RUSAGE_SELF,&usage)==0) {
          const double cpu=usage.ru_utime.tv_sec+usage.ru_utime.tv_usec*1e-6+
                           usage.ru_stime.tv_sec+usage.ru_stime.tv_usec*1e-6;
          if (last_status_>0) number("cpu_percent",100*(cpu-last_cpu_)/interval);
          last_cpu_=cpu;
        }
      }
      std::ifstream memory("/proc/self/statm"); size_t total=0,resident=0;
      if (memory>>total>>resident) number("rss_mib",resident*sysconf(_SC_PAGESIZE)/1048576.0);
      number("lidar_publishers",count_publishers("/avoidance/scan"));
      number("request_publishers",count_publishers("/avoidance/request"));
      number("goal_publishers",count_publishers("/avoidance/goal"));
      if (in.scan) {
        age("lidar_left_age_s",stamp(in.scan->left.header.stamp));
        age("lidar_right_age_s",stamp(in.scan->right.header.stamp));
        number("lidar_receive_age_s",began-in.scan_received);
        number("lidar_left_points",in.scan->left.points.size());
        number("lidar_right_points",in.scan->right.points.size());
      }
      if (in.odom) {
        age("odom_age_s",stamp(in.odom->header.stamp));
        add("odom_frame",in.odom->header.frame_id); add("odom_child_frame",in.odom->child_frame_id);
        number("actual_linear",in.odom->twist.twist.linear.x);
        number("actual_angular",in.odom->twist.twist.angular.z);
      }
      if (in.ultrasonic) {
        number("ultrasonic_age_s",began-in.ultrasonic_received);
        for (int address=0x10;address<0x10+static_cast<int>(ultrasonic_limits_.size());++address) {
          const std::string prefix="us_"+std::to_string(address)+"_";
          int matches=0;
          for (const auto& r:in.ultrasonic->sensors) if (r.bus==bus_ && r.address==address) {
            ++matches;
            number(prefix+"distance_m",r.distance_m); age(prefix+"age_s",stamp(r.stamp));
            add(prefix+"online",r.online?"true":"false"); add(prefix+"valid",r.valid?"true":"false");
            number(prefix+"failures",r.consecutive_failures);
          }
          number(prefix+"matches",matches);
        }
      }
      add("ultrasonic_bus",bus_);
      if (in.request) {
        add("request_source",in.request->source); add("request_session",in.request->session);
        add("request_sequence",std::to_string(in.request->sequence));
        age("request_age_s",stamp(in.request->header.stamp));
        number("request_ttl_s",in.request->valid_for.sec+in.request->valid_for.nanosec*1e-9);
        number("request_linear",in.request->velocity.linear.x); number("request_angular",in.request->velocity.angular.z);
        number("result_linear",decision.velocity.linear); number("result_angular",decision.velocity.angular);
      }
      add("goal_present",in.goal?"true":"false");
      if (in.goal) {
        age("goal_age_s",stamp(in.goal->header.stamp)); add("goal_frame",in.goal->header.frame_id);
        number("goal_x",in.goal->point.x); number("goal_y",in.goal->point.y);
      }
      number("suggestion_linear",suggestion.linear); number("suggestion_angular",suggestion.angular);
      for (const auto& entry:config_values_) add(entry.first,entry.second);
      status_frames_=frames_; status_plans_=plans_; ticks_=0; tick_sum_=0; tick_max_=0;
      status_pub_->publish(status); last_status_=began;
    }
  }
  uint64_t last_planned_frame_=0;
  std::string goal_reason_="idle";
  fa::Velocity last_suggestion_;
  geometry_msgs::msg::PointStamped::ConstSharedPtr last_goal_;
  rclcpp::Time proposal_stamp_{0,0,RCL_ROS_TIME};
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<AvoidanceNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node); executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_avoidance"),"%s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
