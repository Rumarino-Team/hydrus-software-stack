#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include "devices.h"

// --- Macro and enum declarations ---
#define PWM_NEUTRAL   1500
#define PWM_FORWARD   1600
#define PWM_BACKWARDS 1400

#define MOTOR_NUM      8
#define MODEL_NAME     "hydrus"
#define TORPEDO_MOTOR_NUM 2
#define DEPTH_MOTOR_NUM   2

// Provide a global NodeHandle to handle logs (and typical rosserial pub/sub).

const int depth_motors[DEPTH_MOTOR_NUM]   = {5, 6};
const int torpedo_motors[TORPEDO_MOTOR_NUM] = {7, 8};

// A struct for the thruster
struct thrusterArray {
  Thruster thrusters[MOTOR_NUM];
  thrusterArray() : thrusters{
    // forward motor 1
    {true, 1},
    // forward motor 2
    {true, 2},
    // backward motor 1
    {true, 5},
    // backward motor 2
    {false, 6},
    // depth motor 1
    {false, 3},
    // depth motor 2
    {false, 4},
    // torpedo motor 1
    {false, 7},
    // torpedo motor 2
    {true, 8},
  } { }
};

const thrusterArray thrusterArr;

static bool init_motors = false;

// Define your topics
char* thruster_topics_0 = "/" MODEL_NAME "/thrusters/1";
char* thruster_topics_1 = "/" MODEL_NAME "/thrusters/2";
char* thruster_topics_2 = "/" MODEL_NAME "/thrusters/3";
char* thruster_topics_3 = "/" MODEL_NAME "/thrusters/4";
char* depth_topic       = "/" MODEL_NAME "/depth";
char* torpedo_topic     = "/" MODEL_NAME "/torpedo";

// Forward declarations of callback functions
void setThruster_1(const std_msgs::Int8& thrusterValue);
void setThruster_2(const std_msgs::Int8& thrusterValue);
void setThruster_3(const std_msgs::Int8& thrusterValue);
void setThruster_4(const std_msgs::Int8& thrusterValue);
void setDepth(const std_msgs::Int8& thrusterValue);
void launchTorpedo(const std_msgs::Int8& thrusterValue);

void initializeThrustersArduino(void)
{
    init_motors = true;
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        // Set each thruster to neutral
        thrusterArr.thrusters[i].motor.writeMicroseconds(PWM_NEUTRAL);
    }

    // Optional logging to confirm initialization
    nh.loginfo("Thrusters initialized to neutral.");
}

//----------------------
//     setThruster
//----------------------
void setThruster(int id, const std_msgs::Int8& thrusterValue) {
  if (!init_motors) {
    nh.logwarn("Thrusters not initialized yet!");
    return;
  }

  const int data = thrusterValue.data;
  bool forward   = thrusterArr.thrusters[id].forward;
  int value      = 0;

  // Sanity check
  if (data <= -5 || data >= 5) {
    nh.logwarn("Received thruster command out of expected [-4..4] range. Ignoring.");
    return;
  }

  // Compute PWM based on sign
  if (data == 0) {
    value = PWM_NEUTRAL;
  } else if (data < 0) { 
    // Going in reverse direction
    if (forward) {
      value = PWM_BACKWARDS + (1 + data) * 50;
    } else {
      value = PWM_FORWARD - (1 + data) * 50;
    }
  } else {
    // data > 0, going in forward direction
    if (forward) {
      value = PWM_FORWARD + (1 - data) * 50;
    } else {
      value = PWM_BACKWARDS - (1 - data) * 50;
    }
  }

  // Write to servo
  thrusterArr.thrusters[id].motor.writeMicroseconds(value);

  // Add logging to show thruster changes
  char log_buffer[64];
  snprintf(log_buffer, sizeof(log_buffer), 
           "Thruster[%d]: command=%d, PWM=%d, forward=%s",
           id, data, value, forward ? "true" : "false");
  nh.loginfo(log_buffer);
}

//----------------------
//   ROS callbacks
//----------------------
void setThruster_1(const std_msgs::Int8& thrusterValue) {
  setThruster(1, thrusterValue);
}

void setThruster_2(const std_msgs::Int8& thrusterValue) {
  setThruster(2, thrusterValue);
}

void setThruster_3(const std_msgs::Int8& thrusterValue) {
  setThruster(3, thrusterValue);
}

void setThruster_4(const std_msgs::Int8& thrusterValue) {
  setThruster(4, thrusterValue);
}

void setDepth(const std_msgs::Int8& thrusterValue) {
  // Depth motors: 5 and 6
  for (int i = 0; i < DEPTH_MOTOR_NUM; ++i) {
    int id = depth_motors[i];
    setThruster(id, thrusterValue);
  }
}

void launchTorpedo(const std_msgs::Int8& thrusterValue) {
  // Torpedo motors: 7 and 8
  for (int i = 0; i < TORPEDO_MOTOR_NUM; ++i) {
    int id = torpedo_motors[i];
    setThruster(id, thrusterValue);
  }
}