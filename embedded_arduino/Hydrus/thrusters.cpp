#include <Servo.h>
#include <std_msgs/Int8.h>
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

static char* thruster_topics[MOTOR_NUM];
static bool init_motors = false;

char* thruster_topics_0 =  "/" MODEL_NAME "/thrusters/1";
char* thruster_topics_1 =  "/" MODEL_NAME "/thrusters/2";
char* thruster_topics_2 =  "/" MODEL_NAME "/thrusters/3";
char* thruster_topics_3 =  "/" MODEL_NAME "/thrusters/4";
char* depth_topic =  "/" MODEL_NAME "/depth";
char* torpedo_topic =  "/" MODEL_NAME "/torpedo";
char* camera_topic = "/" MODEL_NAME "/camera_motor";

ros::Subscriber<std_msgs::Int8> thruster_sub_1(thruster_topics_0, setThruster_1);
ros::Subscriber<std_msgs::Int8> thruster_sub_2(thruster_topics_1, setThruster_2);
ros::Subscriber<std_msgs::Int8> thruster_sub_3(thruster_topics_2, setThruster_3);
ros::Subscriber<std_msgs::Int8> thruster_sub_4(thruster_topics_3, setThruster_4);
ros::Subscriber<std_msgs::Int8> depth_sub(depth_topic, setDepth);
ros::Subscriber<std_msgs::Int8> torpedo_sub(torpedo_topic, launchTorpedo);
ros::Subscriber<std_msgs::Int8> camera_motor_sub(camera_topic, setCameraMotor);

void initializeThrustersArduino(void)
{
    init_motors = true;
    for (uint8_t i = 0; i < MOTOR_NUM; i++){
        thrusterArr[i].motor.writeMicroseconds(PWM_NEUTRAL);  // This sets the thrusters output force to 0 lbf
    }
}

//----------------------
//----------------------
//  Callbacks

// Setter for the thruster motor PWM values

void setThruster(int id, const std_msgs::Int8& thrusterValue) {
  if(!init_motors) return;

  const int data = thrusterValue.data;
  bool forward = thrusterArr[id-1].forward;
  int value;
 
  value = getPWMValue(data, forward);
  thrusterArr[id-1].motor.writeMicroseconds(value);
}

void setThruster_1(const std_msgs::Int8& thrusterValue)
{
  setThruster(1, thrusterValue);
}
void setThruster_2(const std_msgs::Int8& thrusterValue)
{
  setThruster(2, thrusterValue);
}
void setThruster_3(const std_msgs::Int8& thrusterValue)
{
  setThruster(3, thrusterValue);
}
void setThruster_4(const std_msgs::Int8& thrusterValue)
{
  setThruster(4, thrusterValue);
}

void setDepth(const std_msgs::Int8& thrusterValue)
{
  for (int i = 0; i < DEPTH_MOTOR_NUM; ++i) {
    int id = depth_motors[i];
    setThruster(id, thrusterValue);
  }
}

void launchTorpedo(const std_msgs::Int8& thrusterValue)
{
  for (int i = 0; i < TORPEDO_MOTOR_NUM; ++i) {
    int id = torpedo_motors[i];
    setThruster(id, thrusterValue);
  }
}

void setCameraMotor(const std_msgs::Int8& angle) {
  int data = angle.data;
  int motorMsg = map(0, -60, 60, 0, 180);
  thrusterArr[9].motor.write(data);
}