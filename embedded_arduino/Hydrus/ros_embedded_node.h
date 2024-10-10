

#ifndef ROS_DEPS_H

#define ROS_DEPS_H

#include <ros.h>

void initializeThrustersArduino(void);

extern ros::NodeHandle nh;
void initRosNode(void);
void runRosNode(void);



#endif  
