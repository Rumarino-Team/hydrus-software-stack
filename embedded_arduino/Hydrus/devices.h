#ifndef DEVICES_H
#define DEVICES_H

#define PWM_NEUTRAL 1500  // The thruster's output force is 0 lbf at this value

#include <Arduino.h>
#include <Servo.h>

struct Thruster {
  bool forward;
  int pin;
  Servo motor;

  Thruster(bool _forward, int _pin) {
    forward = _forward;
    pin = _pin;
    motor.attach(_pin);
  }
};

// Make thrusterArr accessible from other files
#define MOTOR_NUM 9
extern Thruster thrusterArr[MOTOR_NUM];

// T100 thrusters declarations
void setThruster_1(int thrusterValue);
void setThruster_2(int thrusterValue);
void setThruster_3(int thrusterValue);
void setThruster_4(int thrusterValue);
void setThruster_5(int thrusterValue);
void setThruster_6(int thrusterValue);
void setDepth(int thrusterValue);
void launchTorpedo(int thrusterValue);
void setCameraMotor(int angle);

// Function that translates input value into PWM
int getPWMValue(int data, bool isForward);

// Initialize the thrusters
void initializeThrustersArduino(void);

#endif
