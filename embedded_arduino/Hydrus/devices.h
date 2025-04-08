#ifndef DEVICES_H
#define DEVICES_H

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

// Change from instantiation to extern declaration
extern ros::NodeHandle nh;

// T100 thrusters declarations
void setThruster_1(const std_msgs::Int8& thrusterValue);
void setThruster_2(const std_msgs::Int8& thrusterValue);
void setThruster_3(const std_msgs::Int8& thrusterValue);
void setThruster_4(const std_msgs::Int8& thrusterValue);
void setThruster_5(const std_msgs::Int8& thrusterValue);
void setThruster_6(const std_msgs::Int8& thrusterValue);
void setDepth(const std_msgs::Int8& thrusterValue);
void launchTorpedo(const std_msgs::Int8& thrusterValue);

extern ros::Subscriber<std_msgs::Int8> thruster_sub_1;
extern ros::Subscriber<std_msgs::Int8> thruster_sub_2;
extern ros::Subscriber<std_msgs::Int8> thruster_sub_3;
extern ros::Subscriber<std_msgs::Int8> thruster_sub_4;
extern ros::Subscriber<std_msgs::Int8> depth_sub;
extern ros::Subscriber<std_msgs::Int8> torpedo_sub;
#endif
