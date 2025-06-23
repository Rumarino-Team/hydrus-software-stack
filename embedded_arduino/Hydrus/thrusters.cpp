#include <Servo.h>
#include "devices.h"

// Macro and enum declarations

#define MOTOR_NUM 9
#define MODEL_NAME "hydrus"

#define TORPEDO_MOTOR_NUM 2
#define DEPTH_MOTOR_NUM 2
// Updated depth and torpedo motor IDs based on the table
const int depth_motors[DEPTH_MOTOR_NUM] = {5, 6};  // Motors 5 and 6 using pins 3 and 4
const int torpedo_motors[TORPEDO_MOTOR_NUM] = {7, 8};  // Motors 7 and 8 using pins 7 and 8

Thruster thrusterArr[] = {
  // Updated thruster array to match thruster numbers with pin numbers
  // Thruster 1 - Pin 1 - Forward
  Thruster(true, 1),
  // Thruster 2 - Pin 2 - Forward
  Thruster(true, 2),
  // Thruster 3 - Pin 5 - Forward
  Thruster(true, 3),
  // Thruster 4 - Pin 6 - Backward
  Thruster(false, 4),
  // Thruster 5 - Pin 3 - Backward
  Thruster(false, 5),
  // Thruster 6 - Pin 4 - Backward
  Thruster(false, 6),
  // Thruster 7 - Pin 7 - Backward
  Thruster(false, 7),
  // Thruster 8 - Pin 8 - Forward
  Thruster(true, 8),
  // Thruster 9 - Pin 9 - Forward (Camera)
  Thruster(true, 9)
};

static bool init_motors = false;

void initializeThrustersArduino(void)
{
    Serial.println("Initializing thrusters...");
    init_motors = true;
    for (uint8_t i = 0; i < MOTOR_NUM; i++){
        thrusterArr[i].motor.writeMicroseconds(PWM_NEUTRAL);  // This sets the thrusters output force to 0 lbf
        Serial.print("Initialized thruster ");
        Serial.print(i+1);
        Serial.print(" (pin ");
        Serial.print(thrusterArr[i].pin);
        Serial.println(") to neutral position");
    }
    Serial.println("All thrusters initialized successfully");
}

//----------------------
//----------------------
//  Callbacks

// Setter for the thruster motor PWM values
void setThruster(int id, int pwmValue) {
  int pwm;
  if(!init_motors) {
    Serial.print("Motors not initialized yet! Ignoring command for thruster ");
    Serial.println(id);
    return;
  }

  if (pwmValue < 1000) pwmValue = 1000;  // Enforce PWM limits
  if (pwmValue > 2000) pwmValue = 2000;
  if (thrusterArr[id-1].forward) {
    pwm = pwmValue;
  } else {
    int diff = PWM_NEUTRAL - pwmValue;
    pwm = PWM_NEUTRAL + diff;
  }

  thrusterArr[id-1].motor.writeMicroseconds(pwm);
}

void setThruster_1(int thrusterValue)
{
  setThruster(1, thrusterValue);
}
void setThruster_2(int thrusterValue)
{
  setThruster(2, thrusterValue);
}
void setThruster_3(int thrusterValue)
{
  setThruster(3, thrusterValue);
}
void setThruster_4(int thrusterValue)
{
  setThruster(4, thrusterValue);
}

void setDepth(int thrusterValue)
{
  for (int i = 0; i < DEPTH_MOTOR_NUM; ++i) {
    int id = depth_motors[i];
    setThruster(id, thrusterValue);
  }
}

void launchTorpedo(int thrusterValue)
{
  for (int i = 0; i < TORPEDO_MOTOR_NUM; ++i) {
    int id = torpedo_motors[i];
    setThruster(id, thrusterValue);
  }
}

void setCameraMotor(int angle) {
  // Map value from -60 to 60 degrees to 0 to 180 for servo
  int motorMsg = map(angle, -60, 60, 0, 180);
  thrusterArr[8].motor.write(motorMsg);  // Fixed array index to 8 (9th element)

  // Serial.print("Camera motor: angle=");
  // Serial.print(angle);
  // Serial.print(", mapped=");
  // Serial.println(motorMsg);
}
