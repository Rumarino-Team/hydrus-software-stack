#include "ros_embedded_node.h"
#include "devices.h"

// Add the actual instantiation of the NodeHandle here
ros::NodeHandle nh;

// Define the subscribers here
ros::Subscriber<std_msgs::Int8> thruster_sub_1("", setThruster_1);
ros::Subscriber<std_msgs::Int8> thruster_sub_2("", setThruster_2);
ros::Subscriber<std_msgs::Int8> thruster_sub_3("", setThruster_3);
ros::Subscriber<std_msgs::Int8> thruster_sub_4("", setThruster_4);
ros::Subscriber<std_msgs::Int8> depth_sub("", setDepth);
ros::Subscriber<std_msgs::Int8> torpedo_sub("", launchTorpedo);

// External topic names defined in thrusters.cpp
extern char* thruster_topics_0;
extern char* thruster_topics_1;
extern char* thruster_topics_2;
extern char* thruster_topics_3;
extern char* depth_topic;
extern char* torpedo_topic;

void initRosNode(void)
{
    nh.initNode();  // Initialize ROS node
    
    // Set topics before subscribing
    thruster_sub_1.topic_ = thruster_topics_0;
    thruster_sub_2.topic_ = thruster_topics_1;
    thruster_sub_3.topic_ = thruster_topics_2;
    thruster_sub_4.topic_ = thruster_topics_3;
    depth_sub.topic_ = depth_topic;
    torpedo_sub.topic_ = torpedo_topic;
    
    nh.subscribe(thruster_sub_1);
    nh.subscribe(thruster_sub_2);
    nh.subscribe(thruster_sub_3);
    nh.subscribe(thruster_sub_4);
    nh.subscribe(depth_sub);
    nh.subscribe(torpedo_sub);
    
    // Initialize thrusters
    initializeThrustersArduino();
    
    // Add initialization log
    nh.loginfo("Hydrus ROS node initialized");
}

void runRosNode(void)
{
    nh.spinOnce();
}
