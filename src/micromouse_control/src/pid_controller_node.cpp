// Two-tier control, tier 1: takes a (target_heading, target_speed)
// setpoint from the floodfill solver and drives the wheels to achieve it.
//
// Outer loop (heading regulation): compares the current yaw against the
// commanded target_heading and produces a yaw-rate correction. Yaw is taken
// from the body IMU (drift-free orientation reference); accumulated wheel
// rotation is kept only as a fallback until the first IMU message arrives,
// since encoder-only yaw drifts badly with wheel slip during turns.
//
// Kinematics: (target_speed, yaw-rate correction) is converted into a
// per-wheel angular velocity feedforward via standard differential-drive
// kinematics.
//
// Inner loop (wheel velocity regulation): each wheel's feedforward target
// is trimmed by its own PID against the actual encoder-measured angular
// velocity, compensating for friction, load, and slip that the kinematic
// feedforward alone can't account for.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "micromouse_msgs/msg/motion_setpoint.hpp"

#include "micromouse_control/pid.hpp"

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
}  // namespace

class PidControllerNode : public rclcpp::Node
{
public:
  PidControllerNode()
  : Node("pid_controller_node")
  {
    wheel_radius_ = this->declare_parameter("wheel_radius", 0.016);
    wheel_separation_ = this->declare_parameter("wheel_separation", 0.08);
    control_rate_hz_ = this->declare_parameter("control_rate_hz", 100.0);
    max_wheel_speed_ = this->declare_parameter("max_wheel_angular_velocity", 40.0);

    double heading_kp = this->declare_parameter("heading_kp", 5.0);
    double heading_ki = this->declare_parameter("heading_ki", 0.0);
    // P-only heading control. The plant from yaw-rate command to yaw is a
    // pure integrator, so proportional feedback alone converges
    // exponentially with no overshoot, and there is no derivative term to
    // amplify into jitter when the control dt shrinks under a low Gazebo
    // real-time factor.
    double heading_kd = this->declare_parameter("heading_kd", 0.0);
    double heading_max_omega = this->declare_parameter("heading_max_omega", 8.0);

    // Caps a single tick's dt in case of a lag spike or sim pause/resume
    // (e.g. clicking the Gazebo pause button) - without this, a single
    // huge dt could slam the PID integral terms and odometry position in
    // one step.
    max_dt_ = this->declare_parameter("max_dt", 0.1);

    left_wheel_name_ = this->declare_parameter("left_wheel_joint_name", std::string("left_wheel_joint"));
    right_wheel_name_ = this->declare_parameter("right_wheel_joint_name", std::string("right_wheel_joint"));

    // The odometry frame is set up to coincide directly with world/maze
    // coordinates (matching wherever sim.launch.py spawns the robot), so
    // the solver can read /odom and use it as the maze position directly
    // without needing a separate frame transform.
    x_ = this->declare_parameter("initial_x", 0.09);
    y_ = this->declare_parameter("initial_y", 0.09);

    heading_pid_ = std::make_unique<micromouse_control::PID>(
      heading_kp, heading_ki, heading_kd, -heading_max_omega, heading_max_omega, -2.0, 2.0);

    setpoint_sub_ = this->create_subscription<micromouse_msgs::msg::MotionSetpoint>(
      "motion_setpoint", 10,
      std::bind(&PidControllerNode::setpoint_callback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&PidControllerNode::joint_state_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      std::bind(&PidControllerNode::imu_callback, this, std::placeholders::_1));

    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "wheel_velocity_controller/commands", 10);
    heading_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("heading_error", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PidControllerNode::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(), "pid_controller_node ready, control rate %.1f Hz", control_rate_hz_);
  }

private:
  void setpoint_callback(const micromouse_msgs::msg::MotionSetpoint::SharedPtr msg)
  {
    target_heading_ = msg->target_heading;
    target_speed_ = msg->target_speed;
    has_setpoint_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Yaw from the IMU orientation quaternion. The IMU is mounted axis-
    // aligned with base_link, so its yaw is the robot's heading directly.
    // Full quaternion->yaw (not the pure-yaw shortcut) so any small
    // roll/pitch the sensor reports doesn't corrupt the heading.
    double x = msg->orientation.x;
    double y = msg->orientation.y;
    double z = msg->orientation.z;
    double w = msg->orientation.w;
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    imu_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    has_imu_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    int left_index = -1;
    int right_index = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == left_wheel_name_) {
        left_index = static_cast<int>(i);
      } else if (msg->name[i] == right_wheel_name_) {
        right_index = static_cast<int>(i);
      }
    }

    if (left_index < 0 || right_index < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "joint_states is missing '%s' or '%s' - skipping this message",
        left_wheel_name_.c_str(), right_wheel_name_.c_str());
      return;
    }

    if (left_index < static_cast<int>(msg->velocity.size()) &&
      right_index < static_cast<int>(msg->velocity.size()))
    {
      left_wheel_velocity_ = msg->velocity[left_index];
      right_wheel_velocity_ = msg->velocity[right_index];
    }

    bool have_positions = left_index < static_cast<int>(msg->position.size()) &&
      right_index < static_cast<int>(msg->position.size());

    if (have_positions) {
      if (!has_position_offset_) {
        // Capture wherever the joints happened to start as the zero
        // reference, rather than assuming the sim initializes them at
        // exactly 0.0.
        left_position_offset_ = msg->position[left_index];
        right_position_offset_ = msg->position[right_index];
        has_position_offset_ = true;
      }

      double left_travel = msg->position[left_index] - left_position_offset_;
      double right_travel = msg->position[right_index] - right_position_offset_;

      // Encoder-derived yaw from accumulated wheel rotation. This is only a
      // fallback for the brief window before the first IMU message arrives;
      // once has_imu_ is set, control_loop() uses imu_yaw_ instead, since
      // encoder yaw drifts with wheel slip during turns.
      encoder_yaw_ = wrap_to_pi(
        (right_travel - left_travel) * wheel_radius_ / wheel_separation_);
    }

    has_joint_state_ = true;
  }

  void control_loop()
  {
    if (!has_joint_state_) {
      return;
    }

    // Measure real elapsed simulated time rather than assuming the
    // nominal 1/control_rate_hz_ period. create_wall_timer always fires
    // on real wall-clock time regardless of use_sim_time, so if Gazebo's
    // real-time factor isn't 1.0 (it commonly isn't), each tick covers a
    // different amount of *simulated* time than the nominal period
    // suggests. this->now(), unlike the wall timer, does respect
    // use_sim_time - using the actual difference here is what keeps
    // position integration and every PID term scaled against what
    // really happened in the simulation, not against a guess.
    rclcpp::Time now = this->now();
    double dt;
    if (!has_last_control_time_) {
      dt = 1.0 / control_rate_hz_;  // no prior timestamp yet - one-time fallback
      has_last_control_time_ = true;
    } else {
      dt = (now - last_control_time_).seconds();
    }
    last_control_time_ = now;

    if (dt <= 0.0) {
      // Sim clock hasn't advanced yet (e.g. before the first /clock
      // message) or ticks fired back-to-back with no progress - skip
      // rather than divide-by-zero or feed a PID a zero/negative dt.
      return;
    }
    dt = std::min(dt, max_dt_);  // cap a stall/lag spike from causing one huge, destabilizing step

    // Heading source: prefer the drift-free IMU yaw, fall back to the
    // encoder-derived yaw only until the first IMU message arrives.
    current_yaw_ = has_imu_ ? imu_yaw_ : encoder_yaw_;
    if (has_imu_ && !logged_imu_) {
      logged_imu_ = true;
      RCLCPP_INFO(this->get_logger(), "IMU heading reference active");
    } else if (!has_imu_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "no /imu yet - using encoder-derived yaw (drifts with wheel slip)");
    }

    // Odometry: integrate the actual measured wheel velocities forward
    // using the encoder-derived yaw (current_yaw_, computed above from
    // accumulated wheel rotation - not re-integrated here, to avoid
    // compounding two separate integration drifts). This only needs
    // wheel feedback, not a motion setpoint, so it runs and publishes
    // even before the solver sends its first command - the solver needs
    // to see a valid starting position on /odom right away.
    double measured_speed = (left_wheel_velocity_ + right_wheel_velocity_) * wheel_radius_ / 2.0;
    x_ += measured_speed * std::cos(current_yaw_) * dt;
    y_ += measured_speed * std::sin(current_yaw_) * dt;
    publish_odometry(measured_speed);

    if (!has_setpoint_) {
      return;
    }

    double heading_error = wrap_to_pi(target_heading_ - current_yaw_);
    double omega_cmd = heading_pid_->compute(heading_error, dt);

    std_msgs::msg::Float64 error_msg;
    error_msg.data = heading_error;
    heading_error_pub_->publish(error_msg);

    double v_left = target_speed_ - omega_cmd * wheel_separation_ / 2.0;
    double v_right = target_speed_ + omega_cmd * wheel_separation_ / 2.0;
    double omega_left_target = v_left / wheel_radius_;
    double omega_right_target = v_right / wheel_radius_;

    // Command the kinematic feedforward wheel velocities DIRECTLY.
    //
    // The downstream interface is a *velocity* command interface, and
    // gz_ros2_control already servos each joint to the commanded velocity
    // essentially ideally. Closing a second velocity PID on top of that
    // servo was structurally oscillatory: the measured wheel velocity
    // converges to the command *including* our own trim, so the trim error
    // feeds back on itself (error -> -previous_trim), and with an integral
    // term that winds into a sustained oscillation. That double velocity
    // loop was the primary source of the wheel jitter and the slip that let
    // odometry race ahead of the real body. A single clean loop -
    // heading P -> unicycle (v, omega) -> wheel velocities -> velocity servo -
    // is both correct for this interface and stable.
    double left_cmd = std::clamp(omega_left_target, -max_wheel_speed_, max_wheel_speed_);
    double right_cmd = std::clamp(omega_right_target, -max_wheel_speed_, max_wheel_speed_);

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = {left_cmd, right_cmd};
    wheel_cmd_pub_->publish(cmd_msg);
  }

  void publish_odometry(double measured_speed)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    // Pure yaw rotation -> quaternion, computed directly rather than
    // pulling in tf2's conversion helpers for a single-axis case.
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(current_yaw_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(current_yaw_ / 2.0);
    odom.twist.twist.linear.x = measured_speed;
    odom.twist.twist.angular.z =
      (right_wheel_velocity_ - left_wheel_velocity_) * wheel_radius_ / wheel_separation_;
    odom_pub_->publish(odom);
  }

  // parameters
  double wheel_radius_;
  double wheel_separation_;
  double control_rate_hz_;
  double max_wheel_speed_;
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  double max_dt_;

  // PIDs
  std::unique_ptr<micromouse_control::PID> heading_pid_;

  // setpoint state
  double target_heading_{0.0};
  double target_speed_{0.0};
  bool has_setpoint_{false};

  // feedback state
  double left_wheel_velocity_{0.0};
  double right_wheel_velocity_{0.0};
  double current_yaw_{0.0};
  double encoder_yaw_{0.0};  // fallback heading until the IMU is up
  double imu_yaw_{0.0};
  bool has_imu_{false};
  bool logged_imu_{false};
  bool has_joint_state_{false};
  double left_position_offset_{0.0};
  double right_position_offset_{0.0};
  bool has_position_offset_{false};

  // odometry state
  double x_{0.0};
  double y_{0.0};

  // measured-dt tracking (see control_loop())
  rclcpp::Time last_control_time_;
  bool has_last_control_time_{false};

  // interfaces
  rclcpp::Subscription<micromouse_msgs::msg::MotionSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidControllerNode>());
  rclcpp::shutdown();
  return 0;
}
