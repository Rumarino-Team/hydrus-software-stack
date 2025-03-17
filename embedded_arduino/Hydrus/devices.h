#ifndef DEVICES_H
#define DEVICES_H

#define PWM_NEUTRAL 1500  // The thruster's output force is 0 lbf at th
#define PWM_FORWARD 1600
#define PWM_BACKWARDS 1400

#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int8.h>

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

// T100 thrusters declarations
void setThruster_1(const std_msgs::Int8& thrusterValue);
void setThruster_2(const std_msgs::Int8& thrusterValue);
void setThruster_3(const std_msgs::Int8& thrusterValue);
void setThruster_4(const std_msgs::Int8& thrusterValue);
void setThruster_5(const std_msgs::Int8& thrusterValue);
void setThruster_6(const std_msgs::Int8& thrusterValue);
void setDepth(const std_msgs::Int8& thrusterValue);
void launchTorpedo(const std_msgs::Int8& thrusterValue);

//Function that translates ROS value into PWM
int getPWMValue(int data, bool isForward);

extern ros::Subscriber<std_msgs::Int8> thruster_sub_1;
extern ros::Subscriber<std_msgs::Int8> thruster_sub_2;
extern ros::Subscriber<std_msgs::Int8> thruster_sub_3;
extern ros::Subscriber<std_msgs::Int8> thruster_sub_4;
extern ros::Subscriber<std_msgs::Int8> depth_sub;
extern ros::Subscriber<std_msgs::Int8> torpedo_sub;
#endif
