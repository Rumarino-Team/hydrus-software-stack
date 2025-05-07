#include <Servo.h>
#include "devices.h"

// Macro and enum declarations

#define MOTOR_NUM 9
#define MODEL_NAME "hydrus"

#define TORPEDO_MOTOR_NUM 2
#define DEPTH_MOTOR_NUM 2
const int depth_motors[DEPTH_MOTOR_NUM] = {5, 6};
const int torpedo_motors[TORPEDO_MOTOR_NUM] = {7, 8};

Thruster thrusterArr[] = {
  //Basically declare whether motor is forward or not, and a pin for each thruster
  //forward motor 1
  Thruster(true, 1),
  //forward motor 2
  Thruster(true, 2),
  //backward motor 1
  Thruster(true, 5),
  //backward motor 2
  Thruster(false, 6),
  //depth motor 1
  Thruster(false, 3),
  //depth motor 2
  Thruster(false, 4),
  //torpedo motor 1
  Thruster(false, 7),
  //torpedo motor 2
  Thruster(true, 8),
  //camera motor
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
        Serial.println(" to neutral position");
    }
    Serial.println("All thrusters initialized successfully");
}

//----------------------
//----------------------
//  Callbacks

// Setter for the thruster motor PWM values
void setThruster(int id, int thrusterValue) {
  if(!init_motors) {
    Serial.print("Motors not initialized yet! Ignoring command for thruster ");
    Serial.println(id);
    return;
  }

  bool forward = thrusterArr[id-1].forward;
  int value;
 
  value = getPWMValue(thrusterValue, forward);
  thrusterArr[id-1].motor.writeMicroseconds(value);
  
  // Debug output (comment out if causing performance issues)
  Serial.print("Thruster ");
  Serial.print(id);
  Serial.print(": data=");
  Serial.print(thrusterValue);
  Serial.print(", PWM=");
  Serial.print(value);
  Serial.print(", forward=");
  Serial.println(forward ? "yes" : "no");
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
  
  Serial.print("Camera motor: angle=");
  Serial.print(angle);
  Serial.print(", mapped=");
  Serial.println(motorMsg);
}