#ifndef SWERVYLIB_PID_H_
#define SWERVYLIB_PID_H_
namespace swervylib {
template <typename CalcType = float, typename TimeType = unsigned long,
          typename InputType = long, typename OutputType = float>
class Pid {
private:
  CalcType kp_;
  CalcType ki_;
  CalcType kd_;
  CalcType integral_;
  InputType previous_error_;
  InputType target_;
  TimeType previous_timestamp_;
  InputType deadband_;

public:
  constexpr Pid(CalcType kp, CalcType ki, CalcType kd, InputType target,
                InputType deadband)
      : kp_(kp), ki_(ki), kd_(kd), integral_(0), previous_error_(0),
        target_(target), previous_timestamp_(0), deadband_(deadband) {}

  OutputType Update(TimeType timestamp, InputType new_value) {
    CalcType error = static_cast<CalcType>(target_ - new_value);
    CalcType dt = static_cast<CalcType>(timestamp - previous_timestamp_);
    CalcType p_term = kp_ * error;
    CalcType d_term = kd_ * ((previous_error_ - error) / dt);
    CalcType i_term = ki_ * integral_;
    integral_ += error * dt;
    previous_error_ = error;
    previous_timestamp_ = timestamp;
    CalcType error_abs = error > 0 ? error : -error;
    if (error_abs < deadband_) {
      return 0;
    }
    return p_term + d_term + i_term;
  }

  void SetTarget(InputType target) {
    if (target != target_) {
      target_ = target;
      integral_ = 0;
    }
  }
};
} // namespace swervylib
#endif
