// Two-tier control, tier 2: the floodfill solver. Decides, cell by cell,
// which direction to move next based on sensed walls and the incremental
// floodfill distance field, and issues MotionSetpoint commands (target
// heading + speed) for tier 1 (pid_controller_node) to execute.
//
// Two operating phases (selected by the `mode` parameter):
//
//   EXPLORE - sense walls as it goes, flooding toward the goal, then
//             (optionally) back to the start, building up the wall map.
//             Modest speed; stops to turn so sensing stays cardinal.
//   RUN     - the "speed run": with the map frozen (no more sensing), it
//             recomputes the floodfill and drives the now-known shortest
//             path fast, chaining straights without stopping and slowing
//             only into the turns.
//
//   mode=explore : explore (and optionally return to start), save the map.
//   mode=run     : load a saved map, skip exploring, do the speed run.
//   mode=auto    : explore -> return to start -> save map -> speed run,
//                  all in one launch.
//
// Low-level state machine, repeated for each cell:
//   SENSE - (explore) update known walls from IR, replan, decide next dir.
//   TURN  - if the next direction requires turning, hold position while
//           the heading PID brings the robot to face it.
//   DRIVE - move forward until the next cell center is reached (detected by
//           longitudinal progress along the travel axis, so lateral drift
//           can't spoof or block arrival), then go back to SENSE.
// Stops (DONE) once the speed run (or, in explore mode, exploration) ends.
//
// Direction/heading convention: +column = heading 0 ("E"), +row = heading
// pi/2 ("N"), matching how the robot is spawned (yaw 0) and how
// generate_maze.py lays out columns/rows - see floodfill.hpp for the
// wall-grid convention this mirrors.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "micromouse_msgs/msg/motion_setpoint.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "micromouse_solver/floodfill.hpp"

using micromouse_solver::Direction;
using micromouse_solver::Floodfill;

namespace
{
double wrap_to_pi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double heading_of(Direction d)
{
  switch (d) {
    case Direction::E: return 0.0;
    case Direction::N: return M_PI / 2.0;
    case Direction::W: return M_PI;
    case Direction::S: return -M_PI / 2.0;
  }
  return 0.0;
}

const char * direction_name(Direction d)
{
  switch (d) {
    case Direction::N: return "N";
    case Direction::E: return "E";
    case Direction::S: return "S";
    case Direction::W: return "W";
  }
  return "?";
}

// Snaps a continuous heading to the nearest cardinal direction - used to
// turn IR sensor orientations (front/left/right relative to current
// heading) into maze directions, and to track which way the robot is
// currently facing.
Direction quantize_heading(double heading)
{
  static constexpr double kCandidates[4] = {0.0, M_PI / 2.0, M_PI, -M_PI / 2.0};
  static constexpr Direction kDirs[4] = {Direction::E, Direction::N, Direction::W, Direction::S};
  int best = 0;
  double best_diff = std::fabs(wrap_to_pi(heading - kCandidates[0]));
  for (int i = 1; i < 4; ++i) {
    double diff = std::fabs(wrap_to_pi(heading - kCandidates[i]));
    if (diff < best_diff) {
      best_diff = diff;
      best = i;
    }
  }
  return kDirs[best];
}

}  // namespace

class SolverNode : public rclcpp::Node
{
public:
  SolverNode()
  : Node("solver_node"), floodfill_(0)
  {
    maze_size_ = this->declare_parameter("maze_size", 16);
    cell_size_ = this->declare_parameter("cell_size", 0.18);
    // Explore deliberately slow: under a low Gazebo real-time factor the IR
    // sensors update sparsely in wall-clock terms, so a high explore speed
    // means few samples per cell and stale wall sensing. The speed run is
    // the timed phase, not this one.
    cruise_speed_ = this->declare_parameter("cruise_speed", 0.12);  // explore speed, m/s
    run_speed_ = this->declare_parameter("run_speed", 0.30);        // speed-run speed, m/s
    // Speed bled off in the last decel_zone_ meters before a cell where the
    // robot will have to turn (or the goal), so it arrives slow and on
    // center instead of overshooting and turning off-axis. Straights are
    // taken at full phase speed and chained without stopping.
    approach_speed_ = this->declare_parameter("approach_speed", 0.08);
    decel_zone_ = this->declare_parameter("decel_zone", 0.06);
    position_tolerance_ = this->declare_parameter("position_tolerance", 0.015);
    heading_tolerance_ = this->declare_parameter("heading_tolerance", 0.05);
    wall_threshold_ = this->declare_parameter("wall_detection_threshold", 0.075);
    open_threshold_ = this->declare_parameter("wall_open_threshold", 0.15);
    control_rate_hz_ = this->declare_parameter("control_rate_hz", 20.0);
    // Generous relative to the nominal cell_size_/speed crossing time, but
    // bounded - if a DRIVE makes no forward progress for this long,
    // something is physically blocking it (an undetected wall, most
    // likely) and the robot must stop relying on dead-reckoning ever
    // resolving on its own and replan instead.
    drive_timeout_s_ = this->declare_parameter("drive_timeout_s", 3.0);

    // Lateral centering. The robot keeps itself centered in the corridor
    // using the SIDE IR ranges (the physical walls) - NOT integrated
    // odometry. Steering off integrated x,y is a positive-feedback trap:
    // the same drift that needs correcting is what's measuring the error,
    // so a phantom drift steers the robot off-axis, which grows the real
    // drift. Referencing the walls breaks that loop - the walls don't drift.
    cross_track_kp_ = this->declare_parameter("cross_track_kp", 2.0);
    max_heading_correction_ = this->declare_parameter("max_heading_correction", 0.25);
    // Deadband: ignore lateral errors smaller than this so noisy/quantized
    // IR samples don't keep nudging the heading and chattering the wheels.
    wall_center_deadband_ = this->declare_parameter("wall_center_deadband", 0.004);
    // A side ray closer than this is treated as a wall to follow.
    side_wall_threshold_ = this->declare_parameter("side_wall_threshold", 0.13);
    // Side IR range when perfectly centered: half a cell minus the sensor's
    // lateral offset from the body center (cell/2 - (base/2 + 0.005)).
    wall_follow_setpoint_ = this->declare_parameter("wall_follow_setpoint", 0.05);

    // Defensive guard: don't trust IR readings for wall-sensing unless
    // actually close to cardinal-aligned, since the sensing math assumes
    // the front/left/right rays point along exact cardinal axes.
    max_sense_misalignment_ = this->declare_parameter("max_sense_misalignment", 0.05);

    // "Knocked off-axis" threshold used DURING a drive. It must sit above the
    // intentional lateral-centering steer (which deliberately points the body
    // up to max_heading_correction_ off the cardinal to recenter), otherwise
    // normal centering is misread as a collision and the robot recovers in a
    // loop. Only a deviation past the steer range is a genuine physical knock.
    max_drive_misalignment_ = this->declare_parameter(
      "max_drive_misalignment", max_heading_correction_ + 0.15);

    // Closed-loop odometry correction. The raw /odom position is pure
    // wheel-velocity dead-reckoning and races ahead of the real body
    // whenever the wheels slip (e.g. pressed against a wall). The walls are
    // a drift-free absolute reference: the front IR measures longitudinal
    // distance-to-wall and the side IR measures lateral offset from corridor
    // center. Each DRIVE tick we nudge a persistent position correction
    // toward those IR-implied absolutes by this gain (a complementary
    // filter); a gain < 1 filters the per-sample IR noise (stddev 0.005 m)
    // instead of snapping the estimate onto every noisy reading.
    odom_corr_kp_ = this->declare_parameter("odom_corr_kp", 0.2);

    // A front/side IR range below this means the body is physically in
    // contact with (or wedged against) a wall - much closer than a centered
    // robot ever sees a side wall (~0.05 m). Used for fast, omnidirectional
    // collision detection so a glancing/side hit stops the robot immediately
    // instead of waiting on the dead-reckoning stall timeout.
    contact_threshold_ = this->declare_parameter("contact_threshold", 0.02);

    // Fast head-on stall detection: while driving toward a wall that is
    // within front-IR range, the range must keep shrinking. If the closest
    // front range stops improving for this long the body has stalled against
    // something - report it and replan, far quicker than drive_timeout_s_
    // (which remains the backstop for open stretches with no wall in view).
    ir_stall_window_s_ = this->declare_parameter("ir_stall_window_s", 0.5);

    // Safety bound on the RECOVER state: if aligning + backing up to the cell
    // center somehow can't complete in this long (e.g. truly jammed), snap the
    // position estimate to the cell center and replan anyway rather than hang.
    recover_timeout_s_ = this->declare_parameter("recover_timeout_s", 2.5);

    // How far behind the cell center RECOVER may reverse while steering itself
    // laterally back to the corridor centerline. Backing up purely along the
    // axis never fixes a lateral offset, so without this the robot resumes
    // still off-center and re-clips the same wall. Kept within the cell
    // (center +-0.09 m) so it doesn't reverse into the previous cell.
    recenter_back_ = this->declare_parameter("recenter_back", 0.06);

    // On-center turns: when the next move out of the target cell is a turn,
    // don't commit it until the robot is laterally centered within this
    // tolerance, creeping forward (front permitting) so the steering loop can
    // finish centering. A nonholonomic robot can only fix lateral offset
    // while moving, so turning off-center is what clips the next wall.
    turn_center_tolerance_ = this->declare_parameter("turn_center_tolerance", 0.008);
    // Bound on that extra centering creep past the cell center, so a cell
    // that can't be centered (e.g. only one wall) can't creep into the wall.
    turn_center_max_creep_ = this->declare_parameter("turn_center_max_creep", 0.03);

    // Visualization publication rate (independent of control rate).
    viz_rate_hz_ = this->declare_parameter("viz_rate_hz", 5.0);

    // Phase / mode configuration.
    mode_ = this->declare_parameter("mode", std::string("auto"));
    return_to_start_ = this->declare_parameter("return_to_start", true);
    map_path_ = this->declare_parameter("map_path", std::string("/tmp/micromouse_map.txt"));

    floodfill_ = Floodfill(maze_size_);
    int center = maze_size_ / 2;
    goal_block_ = {
      {center - 1, center - 1}, {center, center - 1},
      {center - 1, center}, {center, center}};
    floodfill_.set_goal_cells(goal_block_);

    configure_initial_phase();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SolverNode::odom_callback, this, std::placeholders::_1));
    ir_front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "ir_front", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        ir_front_range_ = latest_range(msg);
        has_ir_front_ = true;
      });
    ir_left_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "ir_left", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        ir_left_range_ = latest_range(msg);
        has_ir_left_ = true;
      });
    ir_right_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "ir_right", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        ir_right_range_ = latest_range(msg);
        has_ir_right_ = true;
      });

    setpoint_pub_ = this->create_publisher<micromouse_msgs::msg::MotionSetpoint>("motion_setpoint", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("maze_viz", 10);
    // The IR-corrected position estimate (raw /odom + accumulated correction).
    // This is what the controller and the RViz arrow actually use; publishing
    // it makes the corrected pose observable in RViz/rqt and lets offline
    // tools compare it against ground truth (unlike raw /odom, which keeps its
    // dead-reckoning drift).
    corrected_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_corrected", 10);

    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SolverNode::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(), "solver_node ready, mode=%s, goal block at (%d,%d)",
      mode_.c_str(), center - 1, center - 1);
  }

private:
  enum class Phase { EXPLORE_TO_GOAL, EXPLORE_TO_START, SPEED_RUN, FINISHED };

  // Sets the starting phase from the `mode` parameter. For mode=run we load
  // a previously-saved map and go straight to the speed run; if that load
  // fails there is nothing to run, so we fall back to exploring first.
  void configure_initial_phase()
  {
    if (mode_ == "run") {
      if (floodfill_.load_from_file(map_path_)) {
        floodfill_.set_goal_cells(goal_block_);  // rebuild distances from loaded walls
        start_speed_run();
        RCLCPP_INFO(this->get_logger(), "loaded map from %s", map_path_.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "mode=run but could not load map from %s - exploring first instead",
          map_path_.c_str());
        phase_ = Phase::EXPLORE_TO_GOAL;
        sensing_ = true;
        phase_speed_ = cruise_speed_;
      }
    } else {
      phase_ = Phase::EXPLORE_TO_GOAL;
      sensing_ = true;
      phase_speed_ = cruise_speed_;
    }
  }

  static double latest_range(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
  {
    if (msg->ranges.empty()) {
      return 999.0;
    }
    double r = msg->ranges[0];
    return std::isfinite(r) ? r : 999.0;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_heading_ = 2.0 * std::atan2(qz, qw);  // pure-yaw quaternion -> angle
    has_odom_ = true;
  }

  void control_loop()
  {
    // Always need a pose; only need IR data when this phase actually senses
    // walls (the speed run reads a frozen map, so it can start without
    // waiting on the IR topics).
    if (!has_odom_) {
      return;
    }
    if (sensing_ && (!has_ir_front_ || !has_ir_left_ || !has_ir_right_)) {
      return;
    }
    if (!has_started_) {
      has_started_ = true;
      start_time_ = this->now();
      current_col_ = static_cast<int>(x_ / cell_size_);
      current_row_ = static_cast<int>(y_ / cell_size_);
      current_facing_ = quantize_heading(current_heading_);
    }

    switch (state_) {
      case State::SENSE: do_sense(); break;
      case State::TURN: do_turn(); break;
      case State::DRIVE: do_drive(); break;
      case State::RECOVER: do_recover(); break;
      case State::DONE: do_done(); break;
    }

    publish_corrected_odom();

    int viz_stride = std::max(1, static_cast<int>(control_rate_hz_ / viz_rate_hz_));
    if (++viz_tick_ >= viz_stride) {
      viz_tick_ = 0;
      publish_maze_viz();
    }
  }

  // Publishes the IR-corrected pose estimate (raw odom + accumulated
  // correction, with the drift-free IMU heading) on /odom_corrected.
  void publish_corrected_odom()
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x_ + x_pos_correction_;
    msg.pose.pose.position.y = y_ + y_pos_correction_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.z = std::sin(current_heading_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(current_heading_ / 2.0);
    corrected_odom_pub_->publish(msg);
  }

  void do_sense()
  {
    // Reached the current phase's target cell (the floodfill goal is
    // retargeted per phase, so is_goal() means "phase target reached").
    if (floodfill_.is_goal(current_col_, current_row_)) {
      advance_phase();
      if (phase_ == Phase::FINISHED) {
        state_ = State::DONE;
        return;
      }
      // Phase advanced (goal retargeted); current cell is no longer a goal,
      // so fall through and decide the next move toward the new target.
    }

    // Collision recovery: if heading is more than max_sense_misalignment_ off
    // the nearest cardinal (can happen when a wall collision physically rotates
    // the body), issue an active corrective turn before sensing or routing.
    // Without this the PID would hold whatever heading it last received (the
    // collision-rotated one) and the robot would be stuck forever in SENSE.
    // This check is unconditional (explore and speed-run) because a rotated
    // body can never sense or route reliably regardless of mode.
    {
      Direction snapped = quantize_heading(current_heading_);
      double misalignment = std::fabs(
        wrap_to_pi(heading_of(snapped) - current_heading_));
      if (misalignment > max_sense_misalignment_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "heading %.1f deg off %s - issuing corrective turn",
          misalignment * 180.0 / M_PI, direction_name(snapped));
        publish_setpoint(heading_of(snapped), 0.0);
        return;
      }
    }

    if (sensing_) {
      sense_walls();
    }

    auto next = floodfill_.next_direction(current_col_, current_row_, current_facing_);
    if (!next.has_value()) {
      // Dead-end in the speed run: the frozen map may have stale walls from
      // the explore phase. Re-enable sensing for one cycle, update the map
      // with fresh IR readings, and retry before giving up.
      if (phase_ == Phase::SPEED_RUN && !sensing_) {
        RCLCPP_WARN(
          this->get_logger(),
          "speed-run dead end at (%d,%d) - re-sensing to correct stale map",
          current_col_, current_row_);
        sensing_ = true;
        double misalignment = std::fabs(
          wrap_to_pi(heading_of(quantize_heading(current_heading_)) - current_heading_));
        if (misalignment <= max_sense_misalignment_) {
          sense_walls();
        }
        next = floodfill_.next_direction(current_col_, current_row_, current_facing_);
      }
      if (!next.has_value()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "no reachable open neighbor at (%d,%d) - holding position",
          current_col_, current_row_);
        state_ = State::DONE;
        return;
      }
    }

    if (*next == current_facing_) {
      // Already facing the right way - flow straight into DRIVE without
      // stopping, so straight corridors don't pause cell by cell.
      enter_drive(*next);
    } else {
      target_dir_ = *next;
      state_ = State::TURN;
      publish_setpoint(heading_of(target_dir_), 0.0);
    }
  }

  void sense_walls()
  {
    Direction front_dir = quantize_heading(current_heading_);
    Direction left_dir = quantize_heading(current_heading_ + M_PI / 2.0);
    Direction right_dir = quantize_heading(current_heading_ - M_PI / 2.0);

    auto sense_one = [&](double range, Direction dir) {
      if (range < wall_threshold_) {
        floodfill_.report_wall(current_col_, current_row_, dir);
      } else if (range > open_threshold_ && phase_ == Phase::EXPLORE_TO_GOAL) {
        // Only erase false walls during the forward exploration pass. On the
        // return trip the robot re-visits cells from new angles: sensor noise
        // and lateral offset can push a borderline real-wall reading above the
        // open threshold, which would erase a correct wall and start an
        // oscillation (report_open removes wall → wrong route opens → robot
        // drives into physical wall → report_wall restores it → repeat).
        // Physical walls never disappear, so report_open() is only safe while
        // we are actively building the map for the first time.
        floodfill_.report_open(current_col_, current_row_, dir);
      }
    };
    sense_one(ir_front_range_, front_dir);
    sense_one(ir_left_range_, left_dir);
    sense_one(ir_right_range_, right_dir);
  }

  void do_turn()
  {
    publish_setpoint(heading_of(target_dir_), 0.0);
    double error = std::fabs(wrap_to_pi(heading_of(target_dir_) - current_heading_));
    if (error < heading_tolerance_) {
      current_facing_ = target_dir_;
      enter_drive(target_dir_);
    }
  }

  // Common DRIVE entry point. Records the travel axis and target so do_drive()
  // can measure longitudinal progress, and decides up front whether the robot
  // will have to stop at the target cell (a turn or the goal) so the speed
  // profile knows whether to decelerate into it or blow straight through.
  void enter_drive(Direction dir)
  {
    target_dir_ = dir;
    auto [tc, tr] = micromouse_solver::step(current_col_, current_row_, target_dir_);
    target_col_ = tc;
    target_row_ = tr;

    double h = heading_of(target_dir_);
    travel_x_ = std::cos(h);
    travel_y_ = std::sin(h);
    // Reference the start of THIS cell crossing using the IR-corrected position
    // estimate so that both this cell's progress measurement and the next cell's
    // baseline start from the best available position (not drifted raw odom).
    drive_start_x_ = x_ + x_pos_correction_;
    drive_start_y_ = y_ + y_pos_correction_;

    // Will the robot have to stop when it gets there? It will if the target
    // is the goal, or if the best move *out* of the target cell is not the
    // same direction (a turn). In explore this uses current partial
    // knowledge - only a speed hint, since SENSE re-decides on arrival; in
    // the speed run the map is complete so it is exact.
    auto exit_dir = floodfill_.next_direction(target_col_, target_row_, target_dir_);
    will_stop_at_target_ = floodfill_.is_goal(target_col_, target_row_) ||
      !exit_dir.has_value() || *exit_dir != target_dir_;

    drive_start_time_ = this->now();
    last_progress_time_ = drive_start_time_;
    best_remaining_ = std::numeric_limits<double>::max();
    best_ir_front_ = std::numeric_limits<double>::max();
    last_ir_progress_time_ = drive_start_time_;
    has_ir_progress_ = false;
    state_ = State::DRIVE;
    publish_setpoint(h, phase_speed_);
  }

  // Signed lateral offset from corridor center, from the side IR ranges.
  // Positive = robot is to the LEFT of center (so steer right to fix).
  // Falls back to fewer references as walls disappear, and to "hold
  // heading" (0) in an open cell where there is nothing to center against.
  double lateral_error_from_walls() const
  {
    bool left_wall = ir_left_range_ < side_wall_threshold_;
    bool right_wall = ir_right_range_ < side_wall_threshold_;
    if (left_wall && right_wall) {
      return (ir_right_range_ - ir_left_range_) * 0.5;
    }
    if (left_wall) {
      return wall_follow_setpoint_ - ir_left_range_;
    }
    if (right_wall) {
      return ir_right_range_ - wall_follow_setpoint_;
    }
    return 0.0;
  }

  // ── Sensor-only odometry correction (longitudinal) ──────────────────────
  // The walls are an absolute, drift-free position reference. Whenever any
  // wall sits in a sane window ahead, the front IR fixes the longitudinal
  // position directly. We pull the persistent position correction a fraction
  // (odom_corr_kp_) of the way toward that absolute every tick - a
  // complementary filter that tracks the true body while filtering the
  // per-sample IR noise, instead of the old one-shot snap that only fired at
  // map-known walls (and so almost never during forward exploration).
  // `remaining` is the current along-axis distance-to-go from the estimate.
  void apply_longitudinal_correction(double remaining)
  {
    const double kSensorFwdOffset = 0.04;
    const double kWallHalfThick = 0.006;
    const double kIrRemOffset = 0.5 * cell_size_ - kWallHalfThick - kSensorFwdOffset;
    if (ir_front_range_ <= kWallHalfThick || ir_front_range_ >= cell_size_) {
      return;  // no usable wall in front view
    }
    double remaining_ir = ir_front_range_ - kIrRemOffset;
    // Sanity gate: discard if IR and estimate disagree by more than half a
    // cell. Real wheel-slip drift across one cell is a few cm, well inside
    // this; a half-cell+ disagreement means the ray is on the wrong wall (or a
    // sensor fault), so reject it rather than snap the estimate to it.
    if (std::fabs(remaining_ir - remaining) >= 0.5 * cell_size_) {
      return;
    }
    // drift > 0 means the body is further along than the estimate says; nudge
    // the estimate forward along the travel axis by a fraction of that.
    double drift = remaining - remaining_ir;
    x_pos_correction_ += odom_corr_kp_ * drift * travel_x_;
    y_pos_correction_ += odom_corr_kp_ * drift * travel_y_;
  }

  // ── Sensor-only odometry correction (lateral) ───────────────────────────
  // The side IR ranges fix the perpendicular-to-travel position against the
  // known corridor center, so lateral drift stays bounded and the arrow does
  // not slide sideways. Only acts when a side wall is actually in range; an
  // open cell has nothing absolute to reference. Orthogonal to the
  // longitudinal correction, so the two never fight.
  void apply_lateral_correction()
  {
    bool left_wall = ir_left_range_ < side_wall_threshold_;
    bool right_wall = ir_right_range_ < side_wall_threshold_;
    if (!left_wall && !right_wall) {
      return;
    }
    // Perpendicular (left-hand) unit axis for the current travel direction.
    double perp_x = -travel_y_;
    double perp_y = travel_x_;
    // Corridor center coordinate along that axis (current cell center). The
    // perpendicular cell index does not change during a straight crossing.
    double center_perp = ((current_col_ + 0.5) * cell_size_) * perp_x +
                         ((current_row_ + 0.5) * cell_size_) * perp_y;
    // lateral_error_from_walls() is positive when the body is LEFT of center,
    // i.e. displaced in the +perp direction, so the IR-implied position is
    // center + offset.
    double measured_perp = center_perp + lateral_error_from_walls();
    double est_perp = (x_ + x_pos_correction_) * perp_x + (y_ + y_pos_correction_) * perp_y;
    double drift = measured_perp - est_perp;
    x_pos_correction_ += odom_corr_kp_ * drift * perp_x;
    y_pos_correction_ += odom_corr_kp_ * drift * perp_y;
  }

  void do_drive()
  {
    double path_heading = heading_of(target_dir_);
    double e = lateral_error_from_walls();           // raw, for the centering gate
    double e_db = (std::fabs(e) < wall_center_deadband_) ? 0.0 : e;
    double correction = std::clamp(
      -cross_track_kp_ * e_db, -max_heading_correction_, max_heading_correction_);

    // Longitudinal progress measured relative to the start of this cell
    // crossing, along the travel axis. Uses the IR-corrected position estimate
    // so accumulated wheel-slip drift does not bleed into the remaining counter.
    double traveled = ((x_ + x_pos_correction_) - drive_start_x_) * travel_x_ +
                      ((y_ + y_pos_correction_) - drive_start_y_) * travel_y_;
    double remaining = cell_size_ - traveled;

    // Heading offset from the nearest cardinal: a wall collision can
    // physically rotate the body. Computed before the IR corrections because
    // the wall-referenced fixes assume the rays point along cardinal axes.
    Direction cardinal = quantize_heading(current_heading_);
    double misalignment = std::fabs(wrap_to_pi(heading_of(cardinal) - current_heading_));

    // Closed-loop, sensor-only odometry correction (the "second loop"): pull
    // the persistent position estimate toward the wall-referenced absolutes,
    // then recompute progress against the corrected estimate. Only when the
    // body is cardinal-aligned, so the rays truly measure along-/across-
    // corridor distance. Longitudinal correction is along the travel axis;
    // lateral is perpendicular, so only the longitudinal one changes `traveled`.
    if (misalignment <= max_sense_misalignment_) {
      apply_longitudinal_correction(remaining);
      apply_lateral_correction();
      traveled = ((x_ + x_pos_correction_) - drive_start_x_) * travel_x_ +
                 ((y_ + y_pos_correction_) - drive_start_y_) * travel_y_;
      remaining = cell_size_ - traveled;
    }

    // ── Collision handling: stop, detect, recenter, continue ──────────────
    // Every collision funnels through ONE robust path: record the offending
    // wall, then hand off to RECOVER, which halts the wheels, aligns to the
    // corridor axis, backs up to the CURRENT cell center, and snaps the
    // position estimate there before replanning. Anchoring to a known-good
    // reference (the cell the robot was last centered in) - instead of guessing
    // a cell from drifted odometry - is what fixes the wrong-location problem,
    // and physically freeing the robot from the wall first prevents the
    // re-wedge loops that produced the stream of repeated collision messages.

    // Side contact: a centered robot never sees a side wall nearer than
    // ~0.05 m, so a side range inside contact_threshold_ is a real glancing hit
    // the front IR cannot see.
    if (ir_left_range_ < contact_threshold_ || ir_right_range_ < contact_threshold_) {
      Direction side = (ir_left_range_ < ir_right_range_)
        ? quantize_heading(current_heading_ + M_PI / 2.0)
        : quantize_heading(current_heading_ - M_PI / 2.0);
      floodfill_.report_wall(current_col_, current_row_, side);
      enter_recover("side contact");
      return;
    }

    // Body knocked off the corridor axis (beyond the intentional centering
    // steer): the front ray no longer points along the corridor, so it cannot
    // be trusted as "wall ahead" - recover.
    if (misalignment > max_drive_misalignment_) {
      enter_recover("knocked off-axis");
      return;
    }

    // Front wall, split by how far we have crossed:
    //   ≥ 50%: we are inside the target cell and this is its EXIT wall - a
    //          normal "stop at center and turn", NOT a crash. Record it, force
    //          a decelerated centered arrival, and let SENSE do the turn.
    //   < 50%: the move into the target cell is genuinely blocked - recover.
    if (ir_front_range_ < wall_threshold_) {
      bool crossed = (traveled >= cell_size_ * 0.5);
      if (crossed) {
        if (floodfill_.report_wall(target_col_, target_row_, target_dir_)) {
          RCLCPP_INFO(
            this->get_logger(),
            "wall at exit of (%d,%d) heading %s - decelerating to center, then turning",
            target_col_, target_row_, direction_name(target_dir_));
        }
        will_stop_at_target_ = true;  // decel + on-center arrival below
      } else {
        floodfill_.report_wall(current_col_, current_row_, target_dir_);
        enter_recover("blocked ahead");
        return;
      }
    }

    // Fast head-on stall: while a wall is in front view the range must keep
    // shrinking; if the closest range stalls for ir_stall_window_s_ the body is
    // pressed against it - recover. (drive_timeout_s_ below is the slower
    // backstop for open stretches with no wall in view.)
    if (ir_front_range_ < cell_size_) {
      if (!has_ir_progress_ || ir_front_range_ < best_ir_front_ - 1e-3) {
        best_ir_front_ = ir_front_range_;
        last_ir_progress_time_ = this->now();
        has_ir_progress_ = true;
      } else if ((this->now() - last_ir_progress_time_).seconds() > ir_stall_window_s_) {
        floodfill_.report_wall(current_col_, current_row_, target_dir_);
        enter_recover("front-IR stall");
        return;
      }
    } else {
      has_ir_progress_ = false;  // no wall in view - nothing to track
    }

    // ── On-center turn gating ─────────────────────────────────────────────
    // If a turn (or the goal) is coming at the target, don't declare arrival
    // until the robot is laterally centered, creeping forward (front
    // permitting, bounded by turn_center_max_creep_ past the center) so the
    // steering loop can finish centering. A nonholonomic robot can only fix
    // lateral offset while moving, so a turn taken off-center is what clips
    // the next wall.
    bool centered = std::fabs(e) <= turn_center_tolerance_;
    bool can_creep = (remaining > -turn_center_max_creep_) &&
                     (ir_front_range_ > wall_threshold_);
    bool hold_for_center = will_stop_at_target_ && !centered && can_creep;

    double speed = phase_speed_;
    if ((will_stop_at_target_ || hold_for_center) && remaining < decel_zone_) {
      double frac = std::clamp(remaining / decel_zone_, 0.0, 1.0);
      speed = approach_speed_ + (phase_speed_ - approach_speed_) * frac;
    }
    publish_setpoint(path_heading + correction, speed);

    if (remaining <= position_tolerance_ && !hold_for_center) {
      current_col_ = target_col_;
      current_row_ = target_row_;
      state_ = State::SENSE;
      return;
    }

    // Open-stretch stall backstop (no wall in front view, yet no forward
    // progress for drive_timeout_s_): something is blocking the move - record
    // it in the travel direction and recover, rather than waiting on
    // dead-reckoning to resolve on its own.
    if (remaining < best_remaining_ - 1e-4) {
      best_remaining_ = remaining;
      last_progress_time_ = this->now();
    }
    if ((this->now() - last_progress_time_).seconds() > drive_timeout_s_) {
      floodfill_.report_wall(current_col_, current_row_, target_dir_);
      enter_recover("stalled");
    }
  }

  // Enters the RECOVER state: halts forward motion now and timestamps the
  // recovery so do_recover() can bound how long it runs. The offending wall is
  // recorded by the caller before this is invoked.
  void enter_recover(const char * reason)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "collision: %s at (%d,%d) - stopping, recentering to cell center",
      reason, current_col_, current_row_);
    recover_start_time_ = this->now();
    state_ = State::RECOVER;
    publish_setpoint(heading_of(target_dir_), 0.0);  // halt forward motion immediately
  }

  // Robust, uniform collision recovery, in three simple phases:
  //   1. align to the corridor axis (rotate in place, no forward motion);
  //   2. reverse straight back to the CURRENT cell center;
  //   3. stop, snap the position estimate to that exact cell center, replan.
  // The snap anchors localization to the cell the robot was last centered in -
  // a known-good reference - instead of a cell guessed from drifted odometry,
  // which is what made the robot "assume the wrong location" after a hit. A
  // timeout guarantees it always terminates even if physically jammed.
  void do_recover()
  {
    double h = heading_of(target_dir_);                 // corridor axis to hold
    double herr = wrap_to_pi(h - current_heading_);
    double cx = (current_col_ + 0.5) * cell_size_;
    double cy = (current_row_ + 0.5) * cell_size_;
    double ahead = ((x_ + x_pos_correction_) - cx) * travel_x_ +
                   ((y_ + y_pos_correction_) - cy) * travel_y_;
    bool timed_out = (this->now() - recover_start_time_).seconds() > recover_timeout_s_;

    // Phase 1: not yet aligned - rotate in place, making no forward progress.
    if (std::fabs(herr) > heading_tolerance_ && !timed_out) {
      publish_setpoint(h, 0.0);
      return;
    }
    // Phase 2: reverse back toward the cell center while ALSO steering to the
    // corridor centerline, so the robot physically leaves the wall instead of
    // resuming at the same lateral offset and re-clipping. Continue until both
    // longitudinally at center and laterally centered (or we have backed up the
    // allowed margin). The lateral-steer sign is FLIPPED relative to forward
    // driving: moving backward, a given heading offset shifts the body the
    // opposite way. The cell behind was just traversed, so reversing is safe.
    double e = lateral_error_from_walls();
    bool lat_centered = std::fabs(e) <= turn_center_tolerance_;
    bool need_reverse = (ahead > position_tolerance_) ||
                        (!lat_centered && ahead > -recenter_back_);
    if (need_reverse && !timed_out) {
      double e_db = (std::fabs(e) < wall_center_deadband_) ? 0.0 : e;
      double corr = std::clamp(
        cross_track_kp_ * e_db, -max_heading_correction_, max_heading_correction_);
      publish_setpoint(h + corr, -approach_speed_);
      return;
    }
    // Phase 3: aligned and centered - stop, snap localization to the exact
    // cell center, and replan from a clean, known position.
    publish_setpoint(h, 0.0);
    x_pos_correction_ = cx - x_;
    y_pos_correction_ = cy - y_;
    RCLCPP_INFO(
      this->get_logger(), "recovered to center of (%d,%d)%s - replanning",
      current_col_, current_row_, timed_out ? " (timeout)" : "");
    state_ = State::SENSE;
  }

  // Advances the high-level phase when a phase target cell is reached.
  void advance_phase()
  {
    switch (phase_) {
      case Phase::EXPLORE_TO_GOAL: {
        double elapsed = (this->now() - start_time_).seconds();
        RCLCPP_INFO(
          this->get_logger(), "explore: reached goal at (%d,%d) after %.2f s",
          current_col_, current_row_, elapsed);
        // In auto we must get back to the start before a meaningful speed
        // run; in explore mode the user may also ask to map the return leg.
        if (mode_ == "auto" || return_to_start_) {
          phase_ = Phase::EXPLORE_TO_START;
          floodfill_.set_goal_cells({{0, 0}});
          RCLCPP_INFO(this->get_logger(), "explore: now returning to start (0,0)");
        } else {
          finish_exploration();
        }
        break;
      }
      case Phase::EXPLORE_TO_START:
        RCLCPP_INFO(this->get_logger(), "explore: back at start, mapping complete");
        finish_exploration();
        break;
      case Phase::SPEED_RUN: {
        double run_elapsed = (this->now() - run_start_time_).seconds();
        RCLCPP_INFO(
          this->get_logger(), "speed run complete: reached goal in %.2f s", run_elapsed);
        phase_ = Phase::FINISHED;
        break;
      }
      case Phase::FINISHED:
        break;
    }
  }

  // End of exploration: persist the discovered map, then either start the
  // speed run (auto) or stop (explore).
  void finish_exploration()
  {
    if (!map_path_.empty()) {
      if (floodfill_.save_to_file(map_path_)) {
        RCLCPP_INFO(this->get_logger(), "saved map to %s", map_path_.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "failed to save map to %s", map_path_.c_str());
      }
    }
    if (mode_ == "auto") {
      start_speed_run();
    } else {
      phase_ = Phase::FINISHED;
    }
  }

  void start_speed_run()
  {
    floodfill_.set_goal_cells(goal_block_);  // run target is always the center
    if (floodfill_.distance(current_col_, current_row_) == micromouse_solver::kUnreachable) {
      RCLCPP_ERROR(
        this->get_logger(),
        "speed run aborted: no path to goal from (%d,%d) in known map - "
        "explore map is incomplete or corrupted",
        current_col_, current_row_);
      phase_ = Phase::FINISHED;
      state_ = State::DONE;
      return;
    }
    sensing_ = false;
    phase_speed_ = run_speed_;
    phase_ = Phase::SPEED_RUN;
    run_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "starting speed run at %.2f m/s", run_speed_);
  }

  // ── Floodfill visualizer ────────────────────────────────────────────────
  // Publishes the full maze state as a MarkerArray on /maze_viz so RViz2 can
  // render the live floodfill. Five layers:
  //   cells  – flat colored cubes (green=near-goal, red=far, grey=unknown)
  //   dtext  – distance numbers inside each cell
  //   walls  – all possible wall positions; invisible (alpha=0) when absent
  //   robot  – arrow showing current cell and heading
  //   info   – phase/cell/time text in the top-right corner of the maze
  void publish_maze_viz()
  {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray msg;
    auto now = this->now();

    // Maze physical constants (must match generate_maze.py defaults).
    const double cs = cell_size_;  // cell pitch
    const double wt = 0.012;       // wall thickness
    const double wh = 0.050;       // wall height

    // ── 1. Find max finite distance for colour normalisation ──────────────
    int max_dist = 1;
    for (int r = 0; r < maze_size_; ++r) {
      for (int c = 0; c < maze_size_; ++c) {
        int d = floodfill_.distance(c, r);
        if (d < micromouse_solver::kUnreachable) {
          max_dist = std::max(max_dist, d);
        }
      }
    }

    // Colour helper: t∈[0,1] maps green(0) → yellow(0.5) → red(1).
    auto dist_color = [](float t, float alpha) -> std::array<float, 4> {
      float r_ch = std::min(2.0f * t, 1.0f);
      float g_ch = 0.90f * (1.0f - t);
      float b_ch = std::max(0.25f - t * 0.5f, 0.0f);
      return {r_ch, g_ch, b_ch, alpha};
    };

    // Base marker template to reduce repetition.
    Marker tmpl;
    tmpl.header.frame_id = "odom";
    tmpl.header.stamp = now;
    tmpl.action = Marker::ADD;

    // ── 2. Cell background cubes + distance text ─────────────────────────
    for (int r = 0; r < maze_size_; ++r) {
      for (int c = 0; c < maze_size_; ++c) {
        int id_base = r * maze_size_ + c;
        int d = floodfill_.distance(c, r);
        bool is_robot_cell = (c == current_col_ && r == current_row_);
        bool is_target_cell =
          (state_ == State::DRIVE && c == target_col_ && r == target_row_);

        // ── cell background ──────────────────────────────────────────────
        Marker cell = tmpl;
        cell.ns = "cells";
        cell.id = id_base;
        cell.type = Marker::CUBE;
        cell.pose.position.x = (c + 0.5) * cs;
        cell.pose.position.y = (r + 0.5) * cs;
        cell.pose.position.z = 0.001;
        cell.pose.orientation.w = 1.0;
        cell.scale.x = cs - 0.003;
        cell.scale.y = cs - 0.003;
        cell.scale.z = 0.002;

        if (is_robot_cell) {
          cell.color.r = 0.20f; cell.color.g = 0.85f;
          cell.color.b = 1.00f; cell.color.a = 0.95f;
        } else if (is_target_cell) {
          cell.color.r = 1.00f; cell.color.g = 0.80f;
          cell.color.b = 0.10f; cell.color.a = 0.95f;
        } else if (floodfill_.is_goal(c, r)) {
          cell.color.r = 0.05f; cell.color.g = 1.00f;
          cell.color.b = 0.30f; cell.color.a = 0.95f;
        } else if (d >= micromouse_solver::kUnreachable) {
          cell.color.r = 0.15f; cell.color.g = 0.15f;
          cell.color.b = 0.20f; cell.color.a = 0.70f;
        } else {
          float t = static_cast<float>(d) / static_cast<float>(max_dist);
          auto col = dist_color(t, 0.82f);
          cell.color.r = col[0]; cell.color.g = col[1];
          cell.color.b = col[2]; cell.color.a = col[3];
        }
        msg.markers.push_back(cell);

        // ── distance number ──────────────────────────────────────────────
        if (d < micromouse_solver::kUnreachable) {
          Marker txt = tmpl;
          txt.ns = "dtext";
          txt.id = id_base;
          txt.type = Marker::TEXT_VIEW_FACING;
          txt.pose.position.x = (c + 0.5) * cs;
          txt.pose.position.y = (r + 0.5) * cs;
          txt.pose.position.z = 0.012;
          txt.pose.orientation.w = 1.0;
          txt.scale.z = 0.028;
          txt.color.r = 1.0f; txt.color.g = 1.0f;
          txt.color.b = 1.0f; txt.color.a = 0.75f;
          txt.text = std::to_string(d);
          msg.markers.push_back(txt);
        }
      }
    }

    // ── 3. Wall markers ──────────────────────────────────────────────────
    // Publish ALL possible wall positions (always), using alpha=0 for absent
    // walls. This means cleared walls (report_open) vanish on the next frame
    // without needing DELETEALL (which causes a visible flicker).

    int wall_id = 0;

    // Vertical walls at x = c * cs, for c = 0 … maze_size_.
    // Each spans one row in y; represented by checking the W side of cell c
    // (or the E side of the last column for the outer-right boundary).
    for (int c = 0; c <= maze_size_; ++c) {
      for (int r = 0; r < maze_size_; ++r) {
        bool outer = (c == 0 || c == maze_size_);
        bool present;
        if (c < maze_size_) {
          present = !floodfill_.is_open(c, r, Direction::W);
        } else {
          present = !floodfill_.is_open(maze_size_ - 1, r, Direction::E);
        }

        Marker w = tmpl;
        w.ns = "walls";
        w.id = wall_id++;
        w.type = Marker::CUBE;
        w.pose.position.x = c * cs;
        w.pose.position.y = (r + 0.5) * cs;
        w.pose.position.z = wh / 2.0;
        w.pose.orientation.w = 1.0;
        w.scale.x = wt;
        w.scale.y = cs + wt;
        w.scale.z = wh;
        if (outer) {
          w.color.r = 0.65f; w.color.g = 0.65f;
          w.color.b = 0.70f; w.color.a = present ? 1.0f : 0.0f;
        } else {
          w.color.r = 0.92f; w.color.g = 0.92f;
          w.color.b = 0.95f; w.color.a = present ? 1.0f : 0.0f;
        }
        msg.markers.push_back(w);
      }
    }

    // Horizontal walls at y = r * cs, for r = 0 … maze_size_.
    for (int r = 0; r <= maze_size_; ++r) {
      for (int c = 0; c < maze_size_; ++c) {
        bool outer = (r == 0 || r == maze_size_);
        bool present;
        if (r < maze_size_) {
          present = !floodfill_.is_open(c, r, Direction::S);
        } else {
          present = !floodfill_.is_open(c, maze_size_ - 1, Direction::N);
        }

        Marker w = tmpl;
        w.ns = "walls";
        w.id = wall_id++;
        w.type = Marker::CUBE;
        w.pose.position.x = (c + 0.5) * cs;
        w.pose.position.y = r * cs;
        w.pose.position.z = wh / 2.0;
        w.pose.orientation.w = 1.0;
        w.scale.x = cs + wt;
        w.scale.y = wt;
        w.scale.z = wh;
        if (outer) {
          w.color.r = 0.65f; w.color.g = 0.65f;
          w.color.b = 0.70f; w.color.a = present ? 1.0f : 0.0f;
        } else {
          w.color.r = 0.92f; w.color.g = 0.92f;
          w.color.b = 0.95f; w.color.a = present ? 1.0f : 0.0f;
        }
        msg.markers.push_back(w);
      }
    }

    // ── 4. Robot arrow ───────────────────────────────────────────────────
    if (has_started_) {
      Marker arrow = tmpl;
      arrow.ns = "robot";
      arrow.id = 0;
      arrow.type = Marker::ARROW;
      // Arrow tracks the IR-corrected position estimate. At every turn where
      // a known wall gives an absolute fix, the arrow snaps back to the true
      // position. Between turns it drifts with odom but the error is bounded
      // to one straight segment. The cyan cell shows the solver's cell belief.
      arrow.pose.position.x = x_ + x_pos_correction_;
      arrow.pose.position.y = y_ + y_pos_correction_;
      arrow.pose.position.z = 0.03;
      double h = current_heading_;
      arrow.pose.orientation.z = std::sin(h / 2.0);
      arrow.pose.orientation.w = std::cos(h / 2.0);
      arrow.scale.x = 0.12;
      arrow.scale.y = 0.025;
      arrow.scale.z = 0.025;
      arrow.color.r = 0.10f; arrow.color.g = 0.50f;
      arrow.color.b = 1.00f; arrow.color.a = 1.00f;
      msg.markers.push_back(arrow);
    }

    // ── 5. Phase / info text ─────────────────────────────────────────────
    {
      const char * phase_str = "EXPLORE→GOAL";
      if (phase_ == Phase::EXPLORE_TO_START) phase_str = "RETURN→START";
      if (phase_ == Phase::SPEED_RUN) phase_str = "SPEED RUN";
      if (phase_ == Phase::FINISHED) phase_str = "FINISHED";

      const char * state_str = "SENSE";
      if (state_ == State::TURN) state_str = "TURN";
      if (state_ == State::DRIVE) state_str = "DRIVE";
      if (state_ == State::RECOVER) state_str = "RECOVER";
      if (state_ == State::DONE) state_str = "DONE";

      double elapsed = has_started_ ? (now - start_time_).seconds() : 0.0;
      char buf[128];
      std::snprintf(
        buf, sizeof(buf), "%s | %s | (%d,%d) | %.1fs",
        phase_str, state_str, current_col_, current_row_, elapsed);

      Marker info = tmpl;
      info.ns = "info";
      info.id = 0;
      info.type = Marker::TEXT_VIEW_FACING;
      info.pose.position.x = 0.0;
      info.pose.position.y = maze_size_ * cs + 0.10;
      info.pose.position.z = 0.05;
      info.pose.orientation.w = 1.0;
      info.scale.z = 0.07;
      info.color.r = 1.0f; info.color.g = 1.0f;
      info.color.b = 1.0f; info.color.a = 1.0f;
      info.text = buf;
      msg.markers.push_back(info);
    }

    viz_pub_->publish(msg);
  }

  // ────────────────────────────────────────────────────────────────────────

  void do_done()
  {
    if (!announced_done_) {
      announced_done_ = true;
      double elapsed = (this->now() - start_time_).seconds();
      RCLCPP_INFO(
        this->get_logger(), "finished at (%d,%d), total elapsed %.2f s",
        current_col_, current_row_, elapsed);
    }
    publish_setpoint(current_heading_, 0.0);
  }

  void publish_setpoint(double target_heading, double target_speed)
  {
    micromouse_msgs::msg::MotionSetpoint msg;
    msg.target_heading = target_heading;
    msg.target_speed = target_speed;
    setpoint_pub_->publish(msg);
  }

  enum class State { SENSE, TURN, DRIVE, RECOVER, DONE };

  // parameters
  int maze_size_;
  double cell_size_;
  double cruise_speed_;
  double run_speed_;
  double approach_speed_;
  double decel_zone_;
  double position_tolerance_;
  double heading_tolerance_;
  double wall_threshold_;
  double open_threshold_;
  double control_rate_hz_;
  double drive_timeout_s_;
  double cross_track_kp_;
  double max_heading_correction_;
  double wall_center_deadband_;
  double side_wall_threshold_;
  double wall_follow_setpoint_;
  double max_sense_misalignment_;
  double max_drive_misalignment_;
  double odom_corr_kp_;
  double contact_threshold_;
  double ir_stall_window_s_;
  double recover_timeout_s_;
  double recenter_back_;
  double turn_center_tolerance_;
  double turn_center_max_creep_;
  std::string mode_;
  bool return_to_start_;
  std::string map_path_;

  Floodfill floodfill_;
  std::vector<std::pair<int, int>> goal_block_;

  // odometry feedback
  double x_{0.0};
  double y_{0.0};
  double current_heading_{0.0};
  bool has_odom_{false};

  // IR feedback - defaults to "far" (no wall) until a real reading arrives
  double ir_front_range_{999.0};
  double ir_left_range_{999.0};
  double ir_right_range_{999.0};
  bool has_ir_front_{false};
  bool has_ir_left_{false};
  bool has_ir_right_{false};

  // high-level phase
  Phase phase_{Phase::EXPLORE_TO_GOAL};
  bool sensing_{true};
  double phase_speed_{0.0};
  rclcpp::Time run_start_time_;

  // solver state
  bool has_started_{false};
  rclcpp::Time start_time_;
  rclcpp::Time drive_start_time_;
  rclcpp::Time last_progress_time_;
  rclcpp::Time recover_start_time_;
  double best_remaining_{0.0};
  // Fast IR-based stall tracking: smallest front-IR range seen this crossing
  // while a wall was in view, and when it last improved.
  double best_ir_front_{0.0};
  rclcpp::Time last_ir_progress_time_;
  bool has_ir_progress_{false};
  bool announced_done_{false};
  State state_{State::SENSE};
  int current_col_{0};
  int current_row_{0};
  Direction current_facing_{Direction::E};
  Direction target_dir_{Direction::E};
  int target_col_{0};
  int target_row_{0};
  double travel_x_{1.0};
  double travel_y_{0.0};
  double drive_start_x_{0.0};
  double drive_start_y_{0.0};
  bool will_stop_at_target_{true};
  // Cumulative IR-based position correction. Accumulates each time the front
  // IR provides an absolute fix (known wall ahead of target cell). Applied on
  // top of raw odom wherever corrected position is needed (drive_start_,
  // traveled, visualization arrow) so drift self-corrects at every turn.
  double x_pos_correction_{0.0};
  double y_pos_correction_{0.0};

  // visualization
  double viz_rate_hz_;
  int viz_tick_{0};

  // interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ir_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ir_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ir_right_sub_;
  rclcpp::Publisher<micromouse_msgs::msg::MotionSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolverNode>());
  rclcpp::shutdown();
  return 0;
}
