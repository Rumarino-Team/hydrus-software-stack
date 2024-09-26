#include <Servo.h>
#include <std_msgs/Int8.h>
#include "devices.h"

// Macro and enum declarations

#define PWM_NEUTRAL 1500  // The thruster's output force is 0 lbf at th
#define PWM_FORWARD 1600
#define PWM_BACKWARDS 1400

#define MOTOR_NUM 8
#define MODEL_NAME "hydrus"

#define TORPEDO_MOTOR_NUM 2
#define DEPTH_MOTOR_NUM 2
const int depth_motors[DEPTH_MOTOR_NUM] = {5, 6};
const int torpedo_motors[TORPEDO_MOTOR_NUM] = {7, 8};

struct thrusterArray {
  Thruster thrusters[MOTOR_NUM];

  //Basically declare whether motor is forward or not, and a pin for each thruster
  thrusterArray() : thrusters{
    //forward motor 1
    {true, 1},
    //forward motor 2
    {true, 2},
    //backward motor 1
    {true, 5},
    //backward motor 2
    {false, 6},
    //depth motor 1
    {false, 3},
    //depth motor 2
    {false, 4},
    //torpedo motor 1
    {false, 7},
    //torpedo motor 2
    {true, 8},
  } { }
};

const thrusterArray thrusterArr;

static char* thruster_topics[MOTOR_NUM];
static bool init_motors = false;

char* thruster_topics_0 =  "/" MODEL_NAME "/thrusters/1";
char* thruster_topics_1 =  "/" MODEL_NAME "/thrusters/2";
char* thruster_topics_2 =  "/" MODEL_NAME "/thrusters/3";
char* thruster_topics_3 =  "/" MODEL_NAME "/thrusters/4";
char* depth_topic =  "/" MODEL_NAME "/depth";
char* torpedo_topic =  "/" MODEL_NAME "/torpedo";

ros::Subscriber<std_msgs::Int8> thruster_sub_1(thruster_topics_0, setThruster_1);
ros::Subscriber<std_msgs::Int8> thruster_sub_2(thruster_topics_1, setThruster_2);
ros::Subscriber<std_msgs::Int8> thruster_sub_3(thruster_topics_2, setThruster_3);
ros::Subscriber<std_msgs::Int8> thruster_sub_4(thruster_topics_3, setThruster_4);
ros::Subscriber<std_msgs::Int8> depth_sub(depth_topic, setDepth);
ros::Subscriber<std_msgs::Int8> torpedo_sub(torpedo_topic, launchTorpedo);

void initializeThrustersArduino(void)
{
    init_motors = true;
    for (uint8_t i = 0; i < MOTOR_NUM; i++){
        thrusterArr.thrusters[i].motor.writeMicroseconds(PWM_NEUTRAL);  // This sets the thrusters output force to 0 lbf
    }
}

//----------------------
//----------------------
//  Callbacks

// Setter for the thruster motor PWM values

void setThruster(int id, const std_msgs::Int8& thrusterValue) {
  if(!init_motors) return;

  const int data = thrusterValue.data;
  bool forward = thrusterArr.thrusters[id].forward;
  int value = 0;

  if (data <= -5 || data >= 5) return; //Sensible values please
 
  if (data == 0) {
    value = PWM_NEUTRAL;
  } else if (data < 0) { //If we want to go backwards
    if (forward) {
      /*Since motor goes forward and we want to go backwards, if data == -1, then
      just write PWM_BACKWARDS (-1 + 1 = 0). If data < 1, then substract the PWM_VALUE to that
      based on the data inserted. For example, -1.5 will sum 1400 + (-0.5)*50,
      which equals 1375. */
      value = PWM_BACKWARDS + (1+data)*50;
    } else {
      /*Same idea but swapped since the motor moves backwards by default.
      It is substracted since we are dealing with negative numbers*/
      value = PWM_FORWARD - (1+data)*50;
    }
  } else { //We want to go forward
    if (forward) {
      value = PWM_FORWARD + (1-data)*50;
    } else {
      value = PWM_BACKWARDS - (1-data)*50;
    }
  }
  thrusterArr.thrusters[id].motor.writeMicroseconds(value);
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
