#ifndef MICROMOUSE_CONTROL__PID_HPP_
#define MICROMOUSE_CONTROL__PID_HPP_

#include <algorithm>

namespace micromouse_control
{

// A small discrete PID controller with output clamping and integral
// clamping (anti-windup). The caller computes `error` itself rather than
// passing (setpoint, measurement) - this keeps the class agnostic to
// domain-specific concerns like heading wraparound at +-pi, which the
// caller handles before calling compute().
class PID
{
public:
  PID(
    double kp, double ki, double kd,
    double output_min, double output_max,
    double integral_min, double integral_max)
  : kp_(kp), ki_(ki), kd_(kd),
    output_min_(output_min), output_max_(output_max),
    integral_min_(integral_min), integral_max_(integral_max)
  {
  }

  double compute(double error, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }

    integral_ += error * dt;
    integral_ = clamp(integral_, integral_min_, integral_max_);

    double derivative = 0.0;
    if (has_prev_error_) {
      derivative = (error - prev_error_) / dt;
    }
    prev_error_ = error;
    has_prev_error_ = true;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return clamp(output, output_min_, output_max_);
  }

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    has_prev_error_ = false;
  }

private:
  static double clamp(double value, double lo, double hi)
  {
    return std::max(lo, std::min(hi, value));
  }

  double kp_, ki_, kd_;
  double output_min_, output_max_;
  double integral_min_, integral_max_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool has_prev_error_{false};
};

}  // namespace micromouse_control

#endif  // MICROMOUSE_CONTROL__PID_HPP_
