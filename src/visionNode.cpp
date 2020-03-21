#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "obj_detector/obj_detector.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "visionNode");

    ros::NodeHandle n;

    ROS_INFO_STREAM("Vision node launched");

    // TODO: initialise vision node for continuous operation
}