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

} // namespace swervylib
#endif
