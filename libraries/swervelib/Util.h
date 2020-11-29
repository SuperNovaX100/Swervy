#ifndef UTIL_H_
#define UTIL_H_
namespace swervylib {
template <typename T> T ApplyDeadband(T val, T deadband) {
  if (val < deadband && val > -deadband) {
    return static_cast<T>(0);
  }
  return val;
}
template <typename T>
T ApplyDeadband(T val, T min_val, T max_val, T zero_val = 0) {
  if (val > min_val && val < max_val) {
    return zero_val;
  }
  return val;
}
template <typename T> T Clamp(T val, T min_val, T max_val) {
  if (val > max_val) {
    return max_val;
  }
  if (val < min_val) {
    return min_val;
  }
  return val;
}
template <typename T> T Dist2d(T v0, T v1) { return sqrt(v0 * v0 + v1 * v1); }

double FixAngle(double a) {
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

long GetTargetReduced(double angle_reduced) {
  angle_reduced /= (2.0 * M_PI); // to [-0.5, 0.5]
  if (angle_reduced < 0) {
    angle_reduced += 1.0; // to [0, 1.0]
  }
  long target_reduced = angle_reduced * ENCODER_COUNTS_PER_TURN; // [0, CPR]
  if (target_reduced == ENCODER_COUNTS_PER_TURN) {
    target_reduced = 0; // [0, CPR)
  }
  return target_reduced;
}

long GetNewTarget(long current_enc, long target_reduced) {
  long current_enc_base = current_enc;
  current_enc %= ENCODER_COUNTS_PER_TURN;
  if (current_enc < 0) {
    current_enc += ENCODER_COUNTS_PER_TURN;
  }
  long fl_error = target_reduced - current_enc;
  if (fl_error > ENCODER_COUNTS_PER_TURN / 2.0) {
    fl_error -= ENCODER_COUNTS_PER_TURN;
  } else if (fl_error < -ENCODER_COUNTS_PER_TURN / 2.0) {
    fl_error += ENCODER_COUNTS_PER_TURN;
  }
  long new_target = current_enc_base + fl_error;
  return new_target;
}
template<typename T>
inline T Max(T v0, T v1) {
  if (v0 > v1) {
    return v0;
  }
  return v1;
}


} // namespace swervylib
#endif
