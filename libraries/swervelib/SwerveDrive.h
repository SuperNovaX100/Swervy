#ifndef SWERVE_DRIVE_H_
#define SWERVE_DRIVE_H_
#include "SwerveModule.h"
namespace swervylib {
class SwerveDrive {
private:
  ::swervylib::SwerveModule<USE_FRONT_LEFT> module_fl_;
  ::swervylib::SwerveModule<USE_FRONT_RIGHT> module_fr_;

public:
  SwerveDrive()
      : module_fl_(MOTOR_TURN_FL_PIN, TURN_FL_ENC_A_PIN, TURN_FL_ENC_B_PIN,
                   MOTOR_DRIVE_FL_PIN, false, true, false),
        module_fr_(MOTOR_TURN_FR_PIN, TURN_FR_ENC_A_PIN, TURN_FR_ENC_B_PIN,
                   MOTOR_DRIVE_FR_PIN, false, true, false) {}

  void Setup() {
    module_fl_.Setup();
    module_fr_.Setup();
  }

  void HandleControlSignal(ControlSignal control_signal) {
    constexpr float kSquareDeadband = 0.04;
    float x_val = control_signal.fwd_;
    float y_val = control_signal.strafe_;
    double angle_reduced = atan2(y_val, x_val);
    double speed = ::swervylib::Dist2d(x_val, y_val);
    speed = ::swervylib::ApplyDeadband(speed, 0.2);
    speed = ::swervylib::Clamp(speed, -0.5, 0.5);

    angle_reduced /= (2.0 * M_PI); // to [-0.5, 0.5]
    if (angle_reduced < 0) {
      angle_reduced += 1.0; // to [0, 1.0]
    }
    long target_reduced = angle_reduced * ENCODER_COUNTS_PER_TURN; // [0, CPR]
    if (target_reduced == ENCODER_COUNTS_PER_TURN) {
      target_reduced = 0; // [0, CPR)
    }
    unsigned long t0 = millis();
    bool run = control_signal.turn_ > 0.5f &&
               ((abs(control_signal.fwd_) > kSquareDeadband) ||
                (abs(control_signal.strafe_) > kSquareDeadband));
    module_fl_.Update(t0, target_reduced, speed, run);
    module_fr_.Update(t0, target_reduced, speed, run);
  }

  void ReadEncoderCounts() {
    Serial.print("ENC FL: ");
    Serial.println(module_fl_.GetEncoderCount());
    Serial.print("ENC FR: ");
    Serial.println(module_fr_.GetEncoderCount());
  }
};
} // namespace swervylib
#endif
