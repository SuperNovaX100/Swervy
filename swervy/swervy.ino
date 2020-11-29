#define FRONT_MODULE true
#if FRONT_MODULE
#define USE_FRONT_LEFT true
#define USE_FRONT_RIGHT true
#define USE_BACK_LEFT false
#define USE_BACK_RIGHT false
#else
#define USE_FRONT_LEFT false
#define USE_FRONT_RIGHT false
#define USE_BACK_LEFT true
#define USE_BACK_RIGHT true
#endif

#include "InputHandler.h"
#include "SwerveDrive.h"

swervylib::InputHandler input_handler;
swervylib::SwerveDrive swerve_drive;

void setup() {
  Serial.begin(9600);
  input_handler.Setup();
  swerve_drive.Setup();
}

void loop() {
  //swerve_drive.ReadEncoderCounts();
  swerve_drive.HandleControlSignal1(input_handler.GetControlSignal());
}
