#pragma once

#include <cstdint>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace finav::avoidance {

struct Pose { double x = 0, y = 0, yaw = 0; };
struct Velocity { double linear = 0, angular = 0; };
struct Point { double x, y; };
struct Ray { Point origin, endpoint; };
struct Footprint {
  double xmin = -.25, xmax = .84, ymin = -.45, ymax = .45, height = .5;
};
struct Camera {
  double x = .575, y = .2775, z = .727, yaw = -.5235987756;
  double fx = 0, fy = 0, cx = 0, cy = 0;
};
struct Config {
  double resolution = .05, range = 3.0, memory_seconds = .5;
  double margin = .05, min_height = .05, depth_margin = .1;
  double max_linear = .15, max_angular = .3;
  double reaction_seconds = .15, linear_deceleration = .3, angular_deceleration = .6;
  double intent_lookahead = 1.0;
  int paths = 61;
  void validate() const;
};
struct Depth {
  int width = 0, height = 0;
  std::vector<float> meters;
};
enum class CellState : uint8_t { Unknown, Free, Occupied };
struct Decision { Velocity velocity; std::string reason; };
struct CandidateTrace {
  Velocity velocity;
  double planner_speed = 0, planner_direction = 0;
  std::string stage, reason;
};
struct PlanningTrace {
  std::string original_stage = "not_checked", original_reason;
  std::string search_status = "not_run";
  int attempts = 0;
  std::array<CandidateTrace, 5> candidates;
  int slowdown_attempts = 0;
  double slowdown_scale = 0;
  std::array<std::string, 3> slowdown_reasons;
};

// All times are monotonic seconds; this class is owned by one worker.
class Core {
 public:
  Core(Config config, Footprint footprint);
  ~Core();
  void clear();
  bool observe(const Depth& depth, const Camera& camera, const Pose& pose, double time);
  bool observe_scan(const std::vector<Ray>& rays, const Pose& pose, double time);
  Decision check(Velocity desired, Velocity actual, const Pose& pose, double now,
                 bool require_observed = true) const;
  Decision correct(Velocity intent, Velocity actual, const Pose& pose, double now,
                   bool manual = false, PlanningTrace* trace = nullptr);
  bool approaches(Point obstacle, Velocity velocity) const;
  Velocity plan(Point local_goal, const Pose& pose, double now);
  CellState state(double x, double y, double now) const;
  size_t obstacle_count() const { return obstacles_.size(); }
  const std::vector<Point>& obstacles() const { return obstacles_; }

 private:
  struct Cell { CellState state = CellState::Unknown; double time = -1; };
  struct Planner;
  Config config_;
  Footprint footprint_;
  int side_, origin_x_ = 0, origin_y_ = 0;
  bool centered_ = false;
  std::vector<Cell> cells_, scratch_;
  std::vector<Point> obstacles_;
  std::unique_ptr<Planner> planner_;
  void recenter(const Pose& pose);
  int index(double x, double y) const;
  std::string sweep(Velocity velocity, const Pose& pose, double now, double lookahead = 0,
                    bool require_observed = true) const;
  Decision plan_checked(Point goal, Velocity actual, const Pose& pose, double now,
                        double max_linear, double lookahead, PlanningTrace* trace = nullptr);
};

}  // namespace finav::avoidance
