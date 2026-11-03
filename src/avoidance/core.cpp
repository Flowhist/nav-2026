#include "finav/avoidance/core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>

namespace finav::avoidance {
namespace {
Point world(Point p, const Pose& pose) {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {pose.x + c*p.x - s*p.y, pose.y + s*p.x + c*p.y};
}
Point local(Point p, const Pose& pose) {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {c*(p.x-pose.x) + s*(p.y-pose.y), -s*(p.x-pose.x) + c*(p.y-pose.y)};
}
bool inside(Point p, const Footprint& f, double pad = 0) {
  return p.x >= f.xmin-pad && p.x <= f.xmax+pad &&
         p.y >= f.ymin-pad && p.y <= f.ymax+pad;
}
bool finite(Velocity v) { return std::isfinite(v.linear) && std::isfinite(v.angular); }
double clearance(Point p, const Footprint& f) {
  return std::hypot(std::max({f.xmin-p.x, 0.0, p.x-f.xmax}),
                    std::max({f.ymin-p.y, 0.0, p.y-f.ymax}));
}
}  // namespace

void Config::validate() const {
  for (double v : {resolution, range, memory_seconds, margin, min_height, depth_margin,
                   max_linear, max_angular, reaction_seconds, linear_deceleration,
                   angular_deceleration, intent_lookahead})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid avoidance parameter");
  if (resolution < .03 || resolution > .1 || range < 1 || range > 4 ||
      memory_seconds > 1 || max_linear > .6 || max_angular > .5 ||
      paths < 15 || paths > 121 || paths % 2 == 0 || reaction_seconds > 1 ||
      linear_deceleration < .1 || angular_deceleration < .1 || intent_lookahead > 1.5)
    throw std::invalid_argument("Avoidance parameter outside bounded operating range");
}

struct Core::Planner {
  mrpt::nav::CPTG_DiffDrive_C ptg;
  mrpt::nav::CHolonomicND nd;
  mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput input;
  Planner(const Config& c, const Footprint& f) {
    ptg.loadDefaultParams();
    mrpt::config::CConfigFileMemory settings;
    ptg.saveToConfigFile(settings, "ptg");
    settings.write("ptg", "refDistance", c.range);
    settings.write("ptg", "num_paths", c.paths);
    settings.write("ptg", "resolution", c.resolution);
    settings.write("ptg", "v_max_mps", c.max_linear);
    settings.write("ptg", "w_max_dps", c.max_angular * 180.0 / M_PI);
    settings.write("ptg", "K", 1.0);
    ptg.loadFromConfigFile(settings, "ptg");
    mrpt::math::CPolygon polygon;
    const double m = c.margin + c.resolution * std::sqrt(2.0);
    polygon.AddVertex(f.xmin-m, f.ymin-m);
    polygon.AddVertex(f.xmax+m, f.ymin-m);
    polygon.AddVertex(f.xmax+m, f.ymax+m);
    polygon.AddVertex(f.xmin-m, f.ymax+m);
    ptg.setRobotShape(polygon);
    ptg.initialize("", false);
    input.targets.resize(1);
    input.maxRobotSpeed = 1;
    nd.setAssociatedPTG(&ptg);
    nd.setTargetApproachSlowDownDistance(.2);
  }
};

Core::Core(Config c, Footprint f) : config_(c), footprint_(f) {
  c.validate();
  for (double v : {f.xmin, f.xmax, f.ymin, f.ymax, f.height})
    if (!std::isfinite(v)) throw std::invalid_argument("Invalid footprint");
  if (f.xmin >= f.xmax || f.ymin >= f.ymax || f.height <= c.min_height ||
      std::max({std::abs(f.xmin), std::abs(f.xmax), std::abs(f.ymin), std::abs(f.ymax)}) >= c.range)
    throw std::invalid_argument("Footprint exceeds local grid");
  side_ = static_cast<int>(std::ceil(2*c.range/c.resolution)) + 1;
  cells_.resize(side_*side_);
  scratch_.resize(cells_.size());
  obstacles_.reserve(cells_.size());
  planner_ = std::make_unique<Planner>(c, f);
}
Core::~Core() = default;
void Core::clear() {
  std::fill(cells_.begin(), cells_.end(), Cell{});
  obstacles_.clear();
  centered_ = false;
}
int Core::index(double x, double y) const {
  const int ix = static_cast<int>(std::floor(x/config_.resolution)) - origin_x_;
  const int iy = static_cast<int>(std::floor(y/config_.resolution)) - origin_y_;
  return ix >= 0 && iy >= 0 && ix < side_ && iy < side_ ? iy*side_+ix : -1;
}
CellState Core::state(double x, double y, double now) const {
  const int i = index(x, y);
  if (!centered_ || i < 0 || now < cells_[i].time || now-cells_[i].time >= config_.memory_seconds)
    return CellState::Unknown;
  return cells_[i].state;
}
void Core::recenter(const Pose& p) {
  const int x = static_cast<int>(std::floor(p.x/config_.resolution))-side_/2;
  const int y = static_cast<int>(std::floor(p.y/config_.resolution))-side_/2;
  if (centered_ && x == origin_x_ && y == origin_y_) return;
  std::fill(scratch_.begin(), scratch_.end(), Cell{});
  if (centered_) {
    for (int row=0; row<side_; ++row) for (int col=0; col<side_; ++col) {
      const int old_x = col+x-origin_x_, old_y = row+y-origin_y_;
      if (old_x >= 0 && old_x < side_ && old_y >= 0 && old_y < side_)
        scratch_[row*side_+col] = cells_[old_y*side_+old_x];
    }
  }
  cells_.swap(scratch_);
  origin_x_ = x; origin_y_ = y; centered_ = true;
}

bool Core::observe(const Depth& depth, const Camera& camera, const Pose& pose, double time) {
  if (depth.width < 3 || depth.height < 3 || depth.width > 640 || depth.height > 480 ||
      depth.meters.size() != static_cast<size_t>(depth.width*depth.height) ||
      !std::isfinite(camera.fx) || !std::isfinite(camera.fy) || camera.fx < .1 || camera.fy < .1 ||
      camera.fx>2000 || camera.fy>2000 || !std::isfinite(camera.cx) || !std::isfinite(camera.cy) ||
      camera.cx<0 || camera.cx>=depth.width || camera.cy<0 || camera.cy>=depth.height)
    throw std::invalid_argument("Invalid depth frame or calibration");
  recenter(pose);
  const double r = config_.resolution;
  const Pose mount{camera.x, camera.y, camera.yaw};
  // A column is free only when the sampled collision-height interval is visible.
  // Missing depth, occlusion and out-of-view columns never create free space.
  for (int row=0; row<side_; ++row) for (int col=0; col<side_; ++col) {
    const Point w{(origin_x_+col+.5)*r, (origin_y_+row+.5)*r};
    const Point p = local(w, pose), optical = local(p, mount);
    if (optical.x <= .15 || optical.x > config_.range || inside(p, footprint_)) continue;
    const int u = std::lround(camera.cx-camera.fx*optical.y/optical.x);
    if (u < 1 || u >= depth.width-1) continue;
    bool free = true;
    const int levels = static_cast<int>(std::ceil((footprint_.height+config_.margin-config_.min_height)/r));
    for (int level=0; level<=levels && free; ++level) {
      const double z = config_.min_height + level*r;
      const int v = std::lround(camera.cy-camera.fy*(z-camera.z)/optical.x);
      if (v < 1 || v >= depth.height-1) { free = false; break; }
      for (int dy=-1; dy<=1 && free; ++dy) for (int dx=-1; dx<=1; ++dx) {
        const float d = depth.meters[(v+dy)*depth.width+u+dx];
        if (!std::isfinite(d) || d <= optical.x+config_.depth_margin+r || d > config_.range) {
          free = false; break;
        }
      }
    }
    if (free) cells_[row*side_+col] = {CellState::Free, time};
  }
  // Occupied cells override free-space evidence from the same observation.
  size_t valid_pixels=0;
  for (int v=0; v<depth.height; ++v) for (int u=0; u<depth.width; ++u) {
    const float d = depth.meters[v*depth.width+u];
    if (!std::isfinite(d) || d < .15 || d > config_.range) continue;
    ++valid_pixels;
    const double z = camera.z-(v-camera.cy)*d/camera.fy;
    if (z < config_.min_height || z > footprint_.height+config_.margin) continue;
    const Point p = world({d, -(u-camera.cx)*d/camera.fx}, mount);
    if (inside(p, footprint_)) continue;
    const Point w = world(p, pose);
    const int i = index(w.x, w.y);
    if (i >= 0) cells_[i] = {CellState::Occupied, time};
  }
  obstacles_.clear();
  for (int row=0; row<side_; ++row) for (int col=0; col<side_; ++col) {
    const Cell& cell = cells_[row*side_+col];
    if (cell.state == CellState::Occupied && time-cell.time < config_.memory_seconds)
      obstacles_.push_back({(origin_x_+col+.5)*r, (origin_y_+row+.5)*r});
  }
  return valid_pixels>0;
}

bool Core::observe_scan(const std::vector<Ray>& rays, const Pose& pose, double time) {
  if (rays.empty() || rays.size()>20000) return false;
  for (const auto& ray:rays)
    for (double v:{ray.origin.x,ray.origin.y,ray.endpoint.x,ray.endpoint.y})
      if (!std::isfinite(v) || std::abs(v)>1e6) return false;
  recenter(pose);
  const double r=config_.resolution;
  // Only measured beam segments clear cells. Missing returns never imply free space.
  for (const auto& ray:rays) {
    const double dx=ray.endpoint.x-ray.origin.x,dy=ray.endpoint.y-ray.origin.y;
    const double length=std::hypot(dx,dy);
    if (length<r) continue;
    const double limit=std::min(length-r*std::sqrt(2.0),config_.range+2.0);
    for (double d=0;d<limit;d+=r*.5) {
      const Point p{ray.origin.x+dx*d/length,ray.origin.y+dy*d/length};
      const int i=index(p.x,p.y);
      if (i>=0) cells_[i]={CellState::Free,time};
    }
  }
  // Apply all endpoints after all clearing so overlapping beams cannot erase a hit.
  for (const auto& ray:rays) {
    if (inside(local(ray.endpoint,pose),footprint_)) continue;
    const int i=index(ray.endpoint.x,ray.endpoint.y);
    if (i>=0) cells_[i]={CellState::Occupied,time};
  }
  obstacles_.clear();
  for (int row=0;row<side_;++row) for (int col=0;col<side_;++col) {
    const Cell& cell=cells_[row*side_+col];
    if (cell.state==CellState::Occupied && time-cell.time<config_.memory_seconds)
      obstacles_.push_back({(origin_x_+col+.5)*r,(origin_y_+row+.5)*r});
  }
  return true;
}

bool Core::approaches(Point obstacle, Velocity velocity) const {
  if (!finite(velocity) || !std::isfinite(obstacle.x) || !std::isfinite(obstacle.y)) return true;
  if (std::abs(velocity.linear)+std::abs(velocity.angular) < 1e-6) return false;
  // Sensors can sit inside the conservative rectangular envelope. Signed clearance
  // distinguishes retreat from deeper penetration even when the echo is inside it.
  const auto gap=[this](Point p) {
    const double outside=clearance(p,footprint_);
    return outside>0 ? outside : std::max({footprint_.xmin-p.x,p.x-footprint_.xmax,
                                         footprint_.ymin-p.y,p.y-footprint_.ymax});
  };
  const double before=gap(obstacle);
  const double horizon=config_.reaction_seconds+std::max(
      std::abs(velocity.linear)/config_.linear_deceleration,
      std::abs(velocity.angular)/config_.angular_deceleration);
  const double radius=std::hypot(std::max(std::abs(footprint_.xmin),std::abs(footprint_.xmax)),
                                 std::max(std::abs(footprint_.ymin),std::abs(footprint_.ymax)));
  const int steps=std::max(2,static_cast<int>(std::ceil(horizon*
      (std::abs(velocity.linear)+radius*std::abs(velocity.angular))/(config_.resolution*.25))));
  double previous=before;
  // Compare the whole footprint, including tail swing, throughout the stopping horizon.
  for (int step=1;step<=steps;++step) {
    const double t=horizon*step/steps, angle=velocity.angular*t;
    const Pose future=std::abs(velocity.angular)<1e-6 ? Pose{velocity.linear*t,0,angle} :
        Pose{velocity.linear/velocity.angular*std::sin(angle),
             velocity.linear/velocity.angular*(1-std::cos(angle)),angle};
    const double after=gap(local(obstacle,future));
    if (after < previous-1e-9) return true;
    previous=after;
  }
  // Parallel travel outside the footprint is not an approach. An echo already
  // inside the envelope still needs increasing clearance to permit escape.
  return before <= 1e-9 && previous <= before+1e-9;
}

std::string Core::sweep(Velocity velocity, const Pose& pose, double now, double lookahead,
                        bool require_observed) const {
  if (std::abs(velocity.linear)+std::abs(velocity.angular) < 1e-6) return {};
  if (!require_observed && std::abs(velocity.angular)>1e-6)
    lookahead=std::min(lookahead,std::abs(velocity.linear)*M_PI/2/std::abs(velocity.angular));
  if (require_observed && lookahead > 0 && std::abs(velocity.angular)*lookahead >
      std::abs(velocity.linear)*M_PI/2) return "turn_exceeds_preview";
  double horizon = config_.reaction_seconds + std::max(
      std::abs(velocity.linear)/config_.linear_deceleration,
      std::abs(velocity.angular)/config_.angular_deceleration);
  if (lookahead > 0 && std::abs(velocity.linear) > 1e-6)
    horizon = std::max(horizon, lookahead/std::abs(velocity.linear));
  const double radius = std::hypot(std::max(std::abs(footprint_.xmin), std::abs(footprint_.xmax)),
                                   std::max(std::abs(footprint_.ymin), std::abs(footprint_.ymax)));
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon*
      (std::abs(velocity.linear)+radius*std::abs(velocity.angular))/(config_.resolution*.5))));
  const double pad = config_.margin+config_.resolution*std::sqrt(2.0);
  for (int step=0; step<=steps; ++step) {
    const double t = horizon*step/steps, angle = velocity.angular*t;
    const Point offset = std::abs(velocity.angular) < 1e-6 ? Point{velocity.linear*t, 0} :
        Point{velocity.linear/velocity.angular*std::sin(angle),
              velocity.linear/velocity.angular*(1-std::cos(angle))};
    const Point center = world(offset, pose);
    const Pose future{center.x, center.y, pose.yaw+angle};
    const double r=config_.resolution;
    double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
    for (double x : {footprint_.xmin-pad,footprint_.xmax+pad})
      for (double y : {footprint_.ymin-pad,footprint_.ymax+pad}) {
        const Point corner=world({x,y},future);
        xmin=std::min(xmin,corner.x); xmax=std::max(xmax,corner.x);
        ymin=std::min(ymin,corner.y); ymax=std::max(ymax,corner.y);
      }
    // Rasterize in the observation grid, avoiding false blind slivers caused
    // by moving footprint samples landing in cells whose centers are inside the body.
    for (int ix=static_cast<int>(std::floor(xmin/r)); ix<=static_cast<int>(std::floor(xmax/r)); ++ix)
      for (int iy=static_cast<int>(std::floor(ymin/r)); iy<=static_cast<int>(std::floor(ymax/r)); ++iy) {
        const Point w{(ix+.5)*r,(iy+.5)*r}, p=local(w,future);
        if (!inside(p,footprint_,pad)) continue;
        const auto occupied = state(w.x, w.y, now);
        const Point initial = local(w, pose);
        const double before = clearance(initial, footprint_), after = clearance(p, footprint_);
        // Existing body space is not a blind spot. Outside it, require coverage
        // where motion reduces clearance; receding rear margins cannot veto forward travel.
        if (occupied == CellState::Occupied && (after < 1e-9 ||
            (step > 0 && after <= before+1e-9))) return "obstacle";
        if (require_observed && before > 1e-9 && after < before-1e-9 && occupied != CellState::Free)
          return "unobserved_sweep";
      }
  }
  return {};
}

Decision Core::check(Velocity desired, Velocity actual, const Pose& pose, double now,
                     bool require_observed) const {
  if (!finite(desired) || !finite(actual)) return {{}, "invalid_velocity"};
  if (std::abs(desired.linear) > config_.max_linear || std::abs(desired.angular) > config_.max_angular ||
      std::abs(actual.linear) > .6 || std::abs(actual.angular) > .5)
    return {{}, "speed_limit"};
  if (desired.linear == 0 && desired.angular == 0) return {{}, "stop_requested"};
  auto reason = sweep(actual, pose, now, 0, require_observed);
  if (reason.empty()) reason = sweep(desired, pose, now, 0, require_observed);
  return reason.empty() ? Decision{desired, "pass"} : Decision{{}, reason};
}

Velocity Core::plan(Point goal, const Pose& pose, double now) {
  return plan_checked(goal, {}, pose, now, config_.max_linear, 0).velocity;
}

Decision Core::correct(Velocity intent, Velocity actual, const Pose& pose, double now, bool manual,
                       PlanningTrace* trace) {
  if (trace) *trace = PlanningTrace{};
  const auto record=[trace](Decision decision, const char* stage) {
    if (trace) { trace->original_stage=stage; trace->original_reason=decision.reason; }
    return decision;
  };
  if (!finite(intent) || !finite(actual)) return record({{}, "invalid_velocity"},"input");
  if (intent.linear == 0 && intent.angular == 0) return record({{}, "stop_requested"},"input");
  const double scale = std::min({1.0, config_.max_linear/std::max(std::abs(intent.linear),1e-9),
                                config_.max_angular/std::max(std::abs(intent.angular),1e-9)});
  const Velocity desired{intent.linear*scale,intent.angular*scale};
  // Reverse and in-place rotation retain PASS/STOP semantics; never invent a reverse escape.
  if (desired.linear <= 1e-6) return record(check(desired,actual,pose,now),"direct_check");
  if (std::abs(actual.linear)>.6 || std::abs(actual.angular)>.5) return record({{},"speed_limit"},"actual_motion");
  const auto actual_reason = sweep(actual,pose,now,0);
  if (!actual_reason.empty()) return record({{},actual_reason},"actual_motion");
  const auto preview_reason=sweep(desired,pose,now,config_.intent_lookahead);
  record({{},preview_reason},"preview");
  if (preview_reason.empty())
    return record({desired,scale<1 ? "limited" : "pass"},"preview");
  // A bounded local target follows the input curvature, not a persistent autonomous goal.
  const double heading = std::clamp(desired.angular*config_.intent_lookahead/
                                    desired.linear, -M_PI/3, M_PI/3);
  const double distance=std::min(config_.range*.8,footprint_.xmax+config_.intent_lookahead+.2);
  const Point goal{distance*std::cos(heading),distance*std::sin(heading)};
  const auto planned=plan_checked(goal,actual,pose,now,desired.linear,config_.intent_lookahead,trace);
  if (!manual || preview_reason!="obstacle" || planned.reason=="corrected") return planned;
  // Keep the original curvature; only the reaction/braking sweep governs approach speed.
  int attempt=0;
  for (const double fraction : {.75,.5,.25}) {
    const Velocity candidate{desired.linear*fraction,desired.angular*fraction};
    const auto checked=check(candidate,actual,pose,now);
    if (trace) {
      trace->slowdown_attempts=attempt+1;
      trace->slowdown_reasons.at(attempt)=checked.reason;
    }
    if (checked.reason=="pass") {
      if (trace) trace->slowdown_scale=fraction;
      return {candidate,"slowed_for_obstacle"};
    }
    ++attempt;
  }
  return planned;
}

Decision Core::plan_checked(Point goal, Velocity actual, const Pose& pose, double now,
                            double max_linear, double lookahead, PlanningTrace* trace) {
  if (!std::isfinite(goal.x) || !std::isfinite(goal.y) || std::hypot(goal.x,goal.y) <= .15) {
    if (trace) trace->search_status="goal_reached_or_invalid";
    return {{},"goal_reached_or_invalid"};
  }
  auto& p = *planner_;
  int k; double distance;
  if (!p.ptg.inverseMap_WS2TP(goal.x,goal.y,k,distance)) {
    if (trace) trace->search_status="goal_unmappable";
    return {{},"no_path"};
  }
  p.ptg.initTPObstacles(p.input.obstacles);
  for (const Point& w : obstacles_) {
    if (state(w.x,w.y,now) != CellState::Occupied) continue;
    const Point obstacle = local(w,pose);
    p.ptg.updateTPObstacle(obstacle.x,obstacle.y,p.input.obstacles);
  }
  for (double& d : p.input.obstacles) d = std::clamp(d/config_.range,0.0,1.0);
  const double angle = p.ptg.index2alpha(k);
  p.input.targets[0] = {distance*std::cos(angle),distance*std::sin(angle)};
  Decision rejected{{},"no_safe_path"};
  if (trace) trace->search_status="attempts_exhausted";
  // Keep the MRPT search bounded. A rejected direction is unavailable on the next attempt.
  for (int attempt=0; attempt<5; ++attempt) {
    mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput output;
    p.nd.navigate(p.input,output);
    CandidateTrace* attempt_trace=trace ? &trace->candidates.at(attempt) : nullptr;
    if (trace) {
      ++trace->attempts;
      attempt_trace->stage="planner";
      attempt_trace->planner_speed=output.desiredSpeed;
      attempt_trace->planner_direction=output.desiredDirection;
    }
    if (!std::isfinite(output.desiredSpeed) || !std::isfinite(output.desiredDirection) ||
        output.desiredSpeed<=0) {
      if (trace) {
        trace->search_status=(!std::isfinite(output.desiredSpeed) || !std::isfinite(output.desiredDirection))
            ? "planner_invalid_output" : "planner_no_motion";
        attempt_trace->reason=trace->search_status;
      }
      break;
    }
    const auto direction=p.ptg.alpha2index(output.desiredDirection);
    auto command=p.ptg.directionToMotionCommand(direction);
    const double linear=command->getVelCmdElement(0),angular=command->getVelCmdElement(1);
    double scale=std::clamp(output.desiredSpeed,0.0,1.0);
    if (std::abs(linear)>max_linear) scale=std::min(scale,max_linear/std::abs(linear));
    if (std::abs(angular)>config_.max_angular) scale=std::min(scale,config_.max_angular/std::abs(angular));
    const Velocity candidate{std::clamp(linear*scale,-max_linear,max_linear),
                             std::clamp(angular*scale,-config_.max_angular,config_.max_angular)};
    rejected=check(candidate,actual,pose,now);
    if (attempt_trace) {
      attempt_trace->velocity=candidate;
      attempt_trace->stage="safety_check";
      attempt_trace->reason=rejected.reason;
    }
    if (candidate.linear>1e-6 && rejected.reason=="pass") {
      const auto reason=sweep(candidate,pose,now,lookahead);
      if (attempt_trace) { attempt_trace->stage="preview"; attempt_trace->reason=reason.empty()?"pass":reason; }
      if (reason.empty()) {
        if (trace) trace->search_status="accepted";
        return {candidate,"corrected"};
      }
      rejected={{},reason};
    } else {
      if (attempt_trace && candidate.linear<=1e-6) {
        attempt_trace->stage="direction"; attempt_trace->reason="nonforward_candidate";
      }
      rejected.velocity={};
    }
    p.input.obstacles.at(direction)=0;
  }
  return {{},rejected.reason=="stop_requested" || rejected.reason=="pass" ? "no_safe_path" : rejected.reason};
}
}  // namespace finav::avoidance
