#ifndef SWERVE_DRIVE_H_
#define SWERVE_DRIVE_H_
#include "SwerveModule.h"
namespace swervylib {
class SwerveDrive {
private:
  ::swervylib::SwerveModule<USE_FRONT_LEFT> module_fl_;
  ::swervylib::SwerveModule<USE_FRONT_RIGHT> module_fr_;
  ::swervylib::SwerveModule<USE_BACK_LEFT> module_bl_;
  ::swervylib::SwerveModule<USE_BACK_RIGHT> module_br_;

public:
  SwerveDrive()
      : module_fr_(MOTOR_TURN_FL_PIN, TURN_FL_ENC_A_PIN, TURN_FL_ENC_B_PIN,
                   MOTOR_DRIVE_FL_PIN, true, true, false),
        module_fl_(MOTOR_TURN_FR_PIN, TURN_FR_ENC_A_PIN, TURN_FR_ENC_B_PIN,
                   MOTOR_DRIVE_FR_PIN, true, true, false),
        module_bl_(MOTOR_TURN_FR_PIN, TURN_FR_ENC_A_PIN, TURN_FR_ENC_B_PIN,
                   MOTOR_DRIVE_FR_PIN, true, true, false),
        module_br_(MOTOR_TURN_FL_PIN, TURN_FL_ENC_A_PIN, TURN_FL_ENC_B_PIN,
                   MOTOR_DRIVE_FL_PIN, true, true, false) {}

  void Setup() {
    module_fl_.Setup();
    module_fr_.Setup();
    module_bl_.Setup();
    module_br_.Setup();
  }

  void HandleControlSignal1(ControlSignal control_signal) {
    constexpr double kSquareDeadband = 0.10;
    constexpr double kMaxWheelSpeed = 0.4;
    constexpr double L = 1.0;
    constexpr double W = 1.0;
    const double R = sqrt(L*L + R*R);

    double fwd = control_signal.fwd_;
    double str = control_signal.strafe_;
    double rcw = control_signal.turn_;
    fwd = ::swervylib::ApplyDeadband(fwd, kSquareDeadband);
    str = ::swervylib::ApplyDeadband(str, kSquareDeadband);
    rcw = ::swervylib::ApplyDeadband(rcw, 0.05);
    fwd *= kMaxWheelSpeed;
    strafe *= kMaxWheelSpeed;

    const double a = str - rcw * (L / R);
    const double b = str + rcw * (L / R);
    const double c = fwd - rcw * (W / R);
    const double d = fwd + rcw * (W / R);
    double ws_fr = ::swervylib::Dist2d(b, c);
    double ws_fl = ::swervylib::Dist2d(b, d);
    double ws_bl = ::swervylib::Dist2d(a, d);
    double ws_br = ::swervylib::Dist2d(a, c);
    const double wa_fr = atan2(b, c);// * 180.0 / M_PI; /* -180 to 180 */
    const double wa_fl = atan2(b, d);// * 180.0 / M_PI; /* -180 to 180 */
    const double wa_bl = atan2(a, d);// * 180.0 / M_PI; /* -180 to 180 */
    const double wa_br = atan2(a, c);// * 180.0 / M_PI; /* -180 to 180 */
    const double max_ws = Max(Max(ws_fr, ws_fl), Max(ws_bl, ws_br));
    if (max_ws > kMaxWheelSpeed) {
      ws_fr /= max_ws;
      ws_fl /= max_ws;
      ws_bl /= max_ws;
      ws_br /= max_ws;
    }
    bool enabled = (abs(control_signal.turn_) > kSquareDeadband) ||
                   (abs(control_signal.fwd_)  > kSquareDeadband) ||
                   (abs(control_signal.strafe_) > kSquareDeadband);
    //enabled = false;
    module_fr_.SetWsWa(ws_fr, wa_fr, enabled);
    module_fl_.SetWsWa(ws_fl, wa_fl, enabled);
    module_bl_.SetWsWa(ws_bl, wa_bl, enabled);
    module_br_.SetWsWa(ws_br, wa_br, enabled);
  }

  void HandleControlSignal(ControlSignal control_signal) {
    constexpr float kSquareDeadband = 0.04;
    const float x_val = control_signal.fwd_;
    const float y_val = -control_signal.strafe_;
    double angle_reduced = atan2(y_val, x_val);
    double speed = ::swervylib::Dist2d(x_val, y_val);
    speed = ::swervylib::ApplyDeadband(speed, 0.2);
    speed = ::swervylib::Clamp(speed, -0.3, 0.3);

    angle_reduced /= (2.0 * M_PI); // to [-0.5, 0.5]
    if (angle_reduced < 0) {
      angle_reduced += 1.0; // to [0, 1.0]
    }
    long target_reduced = angle_reduced * ENCODER_COUNTS_PER_TURN; // [0, CPR]
    if (target_reduced == ENCODER_COUNTS_PER_TURN) {
      target_reduced = 0; // [0, CPR)
    }
    unsigned long t0 = millis();
    bool enabled = control_signal.turn_ > 0.5f &&
               ((abs(x_val) > kSquareDeadband) ||
                (abs(y_val) > kSquareDeadband));
    module_fl_.Update(t0, target_reduced, speed, enabled);
    module_fr_.Update(t0, target_reduced, speed, enabled);
    module_bl_.Update(t0, target_reduced, speed, enabled);
    module_br_.Update(t0, target_reduced, speed, enabled);
  }

  void ReadEncoderCounts() {
    Serial.print("ENC FL: ");
    Serial.println(module_fl_.GetEncoderCount());
    Serial.print("ENC FR: ");
    Serial.println(module_fr_.GetEncoderCount());
    Serial.print("ENC BL: ");
    Serial.println(module_bl_.GetEncoderCount());
    Serial.print("ENC BR: ");
    Serial.println(module_br_.GetEncoderCount());
  }
};
} // namespace swervylib
#endif
