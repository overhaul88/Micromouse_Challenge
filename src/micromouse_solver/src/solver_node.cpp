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
    wall_threshold_ = this->declare_parameter("wall_detection_threshold", 0.10);
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
    max_sense_misalignment_ = this->declare_parameter("max_sense_misalignment", 0.15);

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
      case State::DONE: do_done(); break;
    }
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

    if (sensing_) {
      // sense_walls() assumes the front/left/right rays point along exact
      // cardinal axes. With cross-track correction in DRIVE and the
      // heading_tolerance_ check in TURN the heading should already be
      // well-aligned; this skips reporting from a tilted reading rather
      // than feeding a corrupted wall into the floodfill.
      double misalignment = std::fabs(
        wrap_to_pi(heading_of(quantize_heading(current_heading_)) - current_heading_));
      if (misalignment > max_sense_misalignment_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "skipping sense: heading misaligned by %.1f deg", misalignment * 180.0 / M_PI);
        return;
      }
      sense_walls();
    }

    auto next = floodfill_.next_direction(current_col_, current_row_, current_facing_);
    if (!next.has_value()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "no reachable open neighbor at (%d,%d) - holding position", current_col_, current_row_);
      state_ = State::DONE;
      return;
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

    if (ir_front_range_ < wall_threshold_) {
      floodfill_.report_wall(current_col_, current_row_, front_dir);
    }
    if (ir_left_range_ < wall_threshold_) {
      floodfill_.report_wall(current_col_, current_row_, left_dir);
    }
    if (ir_right_range_ < wall_threshold_) {
      floodfill_.report_wall(current_col_, current_row_, right_dir);
    }
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
    // Reference the start of THIS cell crossing, so longitudinal progress is
    // measured over ~one cell and never accumulates absolute odometry drift.
    drive_start_x_ = x_;
    drive_start_y_ = y_;

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

  void do_drive()
  {
    double path_heading = heading_of(target_dir_);
    double e = lateral_error_from_walls();
    if (std::fabs(e) < wall_center_deadband_) {
      e = 0.0;
    }
    double correction = std::clamp(
      -cross_track_kp_ * e, -max_heading_correction_, max_heading_correction_);

    // Longitudinal progress measured relative to the start of this cell
    // crossing, along the travel axis. Lateral drift can't keep this from
    // reaching the cell pitch, and it never carries absolute odom drift.
    double traveled = (x_ - drive_start_x_) * travel_x_ + (y_ - drive_start_y_) * travel_y_;
    double remaining = cell_size_ - traveled;

    double speed = phase_speed_;
    if (will_stop_at_target_ && remaining < decel_zone_) {
      double frac = std::clamp(remaining / decel_zone_, 0.0, 1.0);
      speed = approach_speed_ + (phase_speed_ - approach_speed_) * frac;
    }
    publish_setpoint(path_heading + correction, speed);

    if (remaining <= position_tolerance_) {
      current_col_ = target_col_;
      current_row_ = target_row_;
      state_ = State::SENSE;
      return;
    }

    // Stall recovery: track the best (smallest) remaining seen so far; if it
    // stops improving for drive_timeout_s_, forward progress has stalled
    // (an undetected wall being the likely cause). Treat the direction as
    // walled, stop, and let floodfill route around it.
    if (remaining < best_remaining_ - 1e-4) {
      best_remaining_ = remaining;
      last_progress_time_ = this->now();
    }
    double stalled_for = (this->now() - last_progress_time_).seconds();
    if (stalled_for > drive_timeout_s_) {
      RCLCPP_WARN(
        this->get_logger(),
        "drive stalled heading %s from (%d,%d) for %.2fs - treating as an undetected wall",
        direction_name(target_dir_), current_col_, current_row_, stalled_for);
      floodfill_.report_wall(current_col_, current_row_, target_dir_);
      publish_setpoint(current_heading_, 0.0);
      state_ = State::SENSE;
    }
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
    sensing_ = false;
    phase_speed_ = run_speed_;
    phase_ = Phase::SPEED_RUN;
    run_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "starting speed run at %.2f m/s", run_speed_);
  }

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

  enum class State { SENSE, TURN, DRIVE, DONE };

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
  double control_rate_hz_;
  double drive_timeout_s_;
  double cross_track_kp_;
  double max_heading_correction_;
  double wall_center_deadband_;
  double side_wall_threshold_;
  double wall_follow_setpoint_;
  double max_sense_misalignment_;
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
  double best_remaining_{0.0};
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

  // interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ir_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ir_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ir_right_sub_;
  rclcpp::Publisher<micromouse_msgs::msg::MotionSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolverNode>());
  rclcpp::shutdown();
  return 0;
}
