#ifndef ROS_DEPS_H

#define ROS_DEPS_H

#include <ros.h>

void initializeThrustersArduino(void);

// Declare the NodeHandle as extern so it can be used across multiple files
void initRosNode(void);
void runRosNode(void);



#endif
