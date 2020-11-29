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
  bool is_inverted_;

public:
  SwerveModule(int turn_pin, int turn_enc_a_pin, int turn_enc_b_pin,
               int drive_pin, bool turn_reversed, bool turn_enc_reversed,
               bool drive_reversed)
      : motor_controller_(0.006f, 0, 0, 0, 10),
        motor_turn_(turn_pin, turn_enc_a_pin, turn_enc_b_pin, turn_reversed,
                    turn_enc_reversed),
        motor_drive_(drive_pin, drive_reversed),
        is_inverted_(false) {
  }

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

  void SetWsWa(double speed, double azimuth, boolean enabled) {
    unsigned long t0 = millis();
    long target_reduced = GetTargetReduced(azimuth);
    long current_enc = motor_turn_.GetEncoderCount();
    long new_target = ::swervylib::GetNewTarget(current_enc, target_reduced);
    motor_controller_.SetTarget(new_target);
    double power = motor_controller_.Update(t0, current_enc);
    if (!enabled) {
      power = 0;
      speed = 0;
    }
    motor_turn_.SetPower(power);
    motor_drive_.SetPower(speed);
  }

  void Set(double azimuth, double drive) {
    unsigned long t0 = millis();
    azimuth *= -ENCODER_COUNTS_PER_TURN;
    long azimuth_position = motor_turn_.GetEncoderCount();
    long azimuth_error = azimuth - azimuth_position;
    is_inverted_ = abs(azimuth_error) > (0.25 * ENCODER_COUNTS_PER_TURN);
    if (is_inverted_) {
      azimuth_error -= copysign(0.5 * ENCODER_COUNTS_PER_TURN, azimuth_error);
      drive = -drive;
    }
    float power = motor_controller_.Update(t0, azimuth_position + azimuth_error);
    if (azimuth < 0.05 && azimuth > -0.05 && drive < 0.05 && drive > -0.05) {
      motor_turn_.SetPower(0);
      motor_drive_.SetPower(0);
    }
    motor_turn_.SetPower(power);
    motor_drive_.SetPower(drive);
  }

  long GetEncoderCount() { return motor_turn_.GetEncoderCount(); }
};

template <> class SwerveModule<false> {
  public:
  SwerveModule(int turn_pin, int turn_enc_a_pin, int turn_enc_b_pin,
               int drive_pin, bool turn_reversed, bool turn_enc_reversed,
               bool drive_reversed) {}
  void Setup() {}
  void Update(unsigned long t0, long target_reduced, double speed, bool enabled) {}
  long GetEncoderCount() const { return 0; }
  void SetWsWa(double speed, double azimuth, bool enabled) {};
};

} // namespace swervylib
#endif
