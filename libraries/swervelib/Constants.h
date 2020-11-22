#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace swervylib {
constexpr int PWM_FWD_PIN = 4;
constexpr int PWM_HORIZ_PIN = 8;
constexpr int PWM_TURN_PIN = 5;

constexpr int MOTOR_TURN_FL_PIN = 11;
constexpr int MOTOR_DRIVE_FL_PIN = 10;
constexpr int TURN_FL_ENC_A_PIN = 7;
constexpr int TURN_FL_ENC_B_PIN = 12;

constexpr int MOTOR_TURN_FR_PIN = 9;
constexpr int MOTOR_DRIVE_FR_PIN = 6;
constexpr int TURN_FR_ENC_A_PIN = 2;
constexpr int TURN_FR_ENC_B_PIN = 3;

constexpr long ENCODER_COUNTS_PER_TURN = 1500;

}; // namespace swervylib

#endif
