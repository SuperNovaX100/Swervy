#ifndef INPUT_HANDLER_H_
#define INPUT_HANDLER_H_
#include "Constants.h"
#include "Util.h"
#include <PWM.hpp>

namespace swervylib {

constexpr float kPwmLowerBound = 1000.0f;
constexpr float kPwmUpperBound = 2000.0f;
constexpr float kPwmZeroPoint = (kPwmLowerBound + kPwmUpperBound) / 2.0f;
constexpr float kPwmHalfRange = (kPwmUpperBound - kPwmLowerBound) / 2.0f;
constexpr float kPwmFwdDeadband = 0.04f;
constexpr float kPwmHorizDeadband = 0.04f;
constexpr float kPwmTurnDeadband = 0.04f;
class ControlSignal {
public:
  ControlSignal() = default;
  ControlSignal(float fwd, float strafe, float turn)
      : fwd_(fwd), strafe_(strafe), turn_(turn) {}
  float fwd_;
  float strafe_;
  float turn_;
};

class InputHandler {
private:
  PWM pwm_fwd_;
  PWM pwm_horiz_;
  PWM pwm_turn_;

public:
  InputHandler()
      : pwm_fwd_(PWM_FWD_PIN), pwm_horiz_(PWM_HORIZ_PIN),
        pwm_turn_(PWM_TURN_PIN) {}

  void Setup() {
    pwm_fwd_.begin(true);
    pwm_horiz_.begin(true);
    pwm_turn_.begin(true);
  }

  ControlSignal GetControlSignal() {
    float fwd = swervylib::ApplyDeadband(
        (static_cast<float>(pwm_fwd_.getValue()) - kPwmZeroPoint) /
            kPwmHalfRange,
        kPwmFwdDeadband);
    float horiz = swervylib::ApplyDeadband(
        (static_cast<float>(pwm_horiz_.getValue()) - kPwmZeroPoint) /
            kPwmHalfRange,
        kPwmHorizDeadband);
    float turn = swervylib::ApplyDeadband(
        (static_cast<float>(pwm_turn_.getValue()) - kPwmZeroPoint) /
            kPwmHalfRange,
        kPwmTurnDeadband);
    return ControlSignal(fwd, horiz, turn);
  }
};

} // namespace swervylib

#endif
