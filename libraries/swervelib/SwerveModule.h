#ifndef SWERVE_MODULE_H_
#define SWERVE_MODULE_H_
#include "Motor.h"
#include "Pid.h"
namespace swervylib {

template <bool Exists> class SwerveModule {
private:
  using SwervePid = ::swervylib::Pid<float, unsigned long, long, float>;
  SwervePid motor_controller_;
  ::swervylib::MotorEx motor_turn_;
  ::swervylib::Motor motor_drive_;

public:
  SwerveModule(int turn_pin, int turn_enc_a_pin, int turn_enc_b_pin,
               int drive_pin, bool turn_reversed, bool turn_enc_reversed,
               bool drive_reversed)
      : motor_controller_(0.005f, 0, 0, 0, 10),
        motor_turn_(turn_pin, turn_enc_a_pin, turn_enc_b_pin, turn_reversed,
                    turn_enc_reversed),
        motor_drive_(drive_pin, drive_reversed) {}

  void Setup() {
    motor_turn_.Setup();
    motor_drive_.Setup();
  }

  void Update(unsigned long t0, long target_reduced, double speed, bool enabled) {
    long current_enc = motor_turn_.GetEncoderCount();
    long new_target = ::swervylib::GetNewTarget(current_enc, target_reduced);
    motor_controller_.SetTarget(new_target);
    float power = motor_controller_.Update(t0, current_enc);
    if (!enabled) {
      power = 0;
      speed = 0;
    }
    motor_turn_.SetPower(power);
    motor_drive_.SetPower(speed);
  }

  long GetEncoderCount() { return motor_turn_.GetEncoderCount(); }
};

template <> class SwerveModule<false> {
  SwerveModule(int turn_pin, int turn_enc_a_pin, int turn_enc_b_pin,
               int drive_pin, bool turn_reversed, bool turn_enc_reversed,
               bool drive_reversed) {}
  void Setup() {}
  void Update(unsigned long t0, long target_reduced, bool enabled) {}
  long GetEncoderCount() const { return 0; }
};

} // namespace swervylib
#endif
