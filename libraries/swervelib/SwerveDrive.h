#ifndef SWERVE_DRIVE_H_
#define SWERVE_DRIVE_H_
#include "Motor.h"
#include "Pid.h"
namespace swervylib {
class SwerveDrive {
private:
#if USE_FRONT_LEFT
  swervylib::Pid<float, unsigned long, long, float> motor_controller_fl_;
  swervylib::MotorEx motor_turn_front_left_;
  swervylib::Motor motor_drive_front_left_;
#endif
#if USE_FRONT_RIGHT
  swervylib::Pid<float, unsigned long, long, float> motor_controller_fr_;
  swervylib::MotorEx motor_turn_front_right_;
  swervylib::Motor motor_drive_front_right_;
#endif
public:
  SwerveDrive()
      :
#if USE_FRONT_LEFT
#ifdef NEED_COMMA
        ,
#else
#define NEED_COMMA
#endif
        motor_controller_fl_(0.005f, 0, 0, 0, 10),
        motor_turn_front_left_(MOTOR_TURN_FL_PIN, TURN_FL_ENC_A_PIN,
                               TURN_FL_ENC_B_PIN, false, true),
        motor_drive_front_left_(MOTOR_DRIVE_FL_PIN)
#endif
#if USE_FRONT_RIGHT
#ifdef NEED_COMMA
        ,
#else
#define NEED_COMMA
#endif
        motor_controller_fr_(0.005f, 0, 0, 0, 10),
        motor_turn_front_right_(MOTOR_TURN_FR_PIN, TURN_FR_ENC_A_PIN,
                                TURN_FR_ENC_B_PIN, false, true),
        motor_drive_front_right_(MOTOR_DRIVE_FR_PIN)
#endif
#undef NEED_COMMA
  {
  }

  void Setup() {
#if USE_FRONT_LEFT
    motor_turn_front_left_.Setup();
    motor_drive_front_left_.Setup();
#endif
#if USE_FRONT_RIGHT
    motor_turn_front_right_.Setup();
    motor_drive_front_right_.Setup();
#endif
  }
  long ComputeControlAngle(long current_enc, long target_reduced) {
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

  void HandleControlSignal(ControlSignal control_signal) {
    constexpr float kSquareDeadband = 0.04;
    float x_val = control_signal.fwd_;
    float y_val = control_signal.strafe_;
    double angle_reduced = atan2(y_val, x_val);
    double speed = sqrt(control_signal.fwd_ * control_signal.fwd_ +
                        control_signal.strafe_ * control_signal.strafe_);

    if (speed < 0.2 && speed > -0.2) {
      speed = 0;
    }
    if (speed > 0.5) {
      speed = 0.5;
    } else if (speed < -0.5) {
      speed = -0.5;
    }

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
#if USE_FRONT_LEFT
    long current_enc_fl = motor_turn_front_left_.GetEncoderCount();
    long new_target_fl = ComputeControlAngle(current_enc_fl, target_reduced);
    motor_controller_fl_.SetTarget(new_target_fl);
    float power_fl = motor_controller_fl_.Update(t0, current_enc_fl);
    if (run) {
      motor_turn_front_left_.SetPower(power_fl);
      motor_drive_front_left_.SetPower(speed);
    } else {
      motor_turn_front_left_.SetPower(0);
      motor_drive_front_left_.SetPower(0);
    }
#endif

#if USE_FRONT_RIGHT
    long current_enc_fr = motor_turn_front_right_.GetEncoderCount();
    long new_target_fr = ComputeControlAngle(current_enc_fr, target_reduced);
    motor_controller_fr_.SetTarget(new_target_fr);
    float power_fr = motor_controller_fr_.Update(t0, current_enc_fr);
    if (run) {
      motor_turn_front_right_.SetPower(power_fr);
      motor_drive_front_right_.SetPower(speed);
    } else {
      motor_turn_front_right_.SetPower(0);
      motor_drive_front_right_.SetPower(0);
    }
#endif
  }

  void ReadEncoderCounts() {
#if USE_FRONT_LEFT
    Serial.print("ENC FL: ");
    Serial.println(motor_turn_front_left_.GetEncoderCount());
#endif
#if USE_FRONT_RIGHT
    Serial.print("ENC FR: ");
    Serial.println(motor_turn_front_right_.GetEncoderCount());
#endif
  }
};
} // namespace swervylib
#endif
