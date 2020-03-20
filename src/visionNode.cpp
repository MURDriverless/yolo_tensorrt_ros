#include "ros/ros.h"
#include "std_msg/String.h"
#include <sstream>
// #include "vision/vision.h" // TODO: add after strucutre is implemented

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "visionNode");

    ros::NodeHandle n;

    ROS_INFO_STREAM("Vision node launched");

    // TODO: initialise vision node for continuous operation
}