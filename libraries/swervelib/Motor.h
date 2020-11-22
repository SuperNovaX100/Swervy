#ifndef MOTOR_H_
#define MOTOR_H_
#include <QuadratureEncoder.h>
#include <Servo.h>

namespace swervylib {

int CalculateServoSignal(float percent_power) {
  if (percent_power > 1.0f) {
    percent_power = 1.0f;
  } else if (percent_power < -1.0f) {
    percent_power = -1.0f;
  } else if (percent_power < 0.05 && percent_power >= 0) {
    return 0;
  } else if (percent_power > -0.05 && percent_power <= 0) {
    return 90;
  }
  if (percent_power > 0.0) {
    return (84 * (1.0 - percent_power)) + 6;
  } else {
    return (84 * -percent_power) + 90;
  }
};

class Motor {
protected:
  Servo motor_;
  int motor_pin_;
  bool reversed_;

public:
  Motor(int motor_pin, bool reversed = false)
      : motor_pin_(motor_pin), reversed_(reversed) {}

  void Setup() { motor_.attach(motor_pin_); }

  void SetPower(float power) {
    if (reversed_) {
      power *= -1;
    }
    int sig = CalculateServoSignal(power);
    motor_.write(sig);
  }
};

class MotorEx : public Motor {
private:
  Encoders encoder_;
  bool reversed_encoder_;

public:
  MotorEx(int motor_pin, int channel_a_pin, int channel_b_pin,
          bool reversed = false, bool reversed_encoder = false)
      : Motor(motor_pin, reversed), encoder_(channel_a_pin, channel_b_pin),
        reversed_encoder_(reversed_encoder) {}

  long GetEncoderCount() {
    long enc_count = encoder_.getEncoderCount();

    if (reversed_) {
      enc_count *= -1;
    }
    if (reversed_encoder_) {
      enc_count *= -1;
    }
    return enc_count;
  }
};

} // namespace swervylib
#endif
