
#include "ros_embedded_node.h"
#include "devices.h"

ros::NodeHandle nh;

void initRosNode(void)
{
    nh.initNode();  // Initialize ROS node
    nh.subscribe(thruster_sub_1);
    nh.subscribe(thruster_sub_2);
    nh.subscribe(thruster_sub_3);
    nh.subscribe(thruster_sub_4);
    nh.subscribe(depth_sub);
    nh.subscribe(torpedo_sub);
}

void runRosNode(void)
{
    nh.spinOnce();
}
