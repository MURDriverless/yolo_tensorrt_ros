#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/package.h>

#include <sstream>

#include "class_detector.h"

#include <chrono>
#include <ctime>
#include <ratio>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  // construct path to the configs
  std::string path = ros::package::getPath("yolo_tensorrt_ros");
  std::string cfg_path = path + "/src/config/yolov3-tiny.cfg";
  std::string weights_path = path + "/src/config/yolov3-tiny.weights";
  std::string img_path = path + "/src/config/dog.jpg";

  Detector detector;
  Config config;
  config.file_model_cfg = cfg_path;
  config.file_model_weights = weights_path;
  config.inference_precison = FP16;
  config.detect_thresh = 0.3;
  detector.init(config);

  cv::Mat mat_image = cv::imread(img_path, cv::IMREAD_UNCHANGED);

  std::vector<Result> res;

  auto t0 = std::chrono::high_resolution_clock::now();
  detector.detect(mat_image, res);
  auto t1 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "publishing yolo results " << count << std::endl;
    ss << "inference duration = " << time_span.count() << " seconds." << std::endl;
    // ss << cfg_path << std::endl;
    // ss << weights_path << std::endl;
    // ss << img_path << std::endl;
    
    // publish detection results
    for (const auto &r : res)
    {
      ss << "id:" << r.id << " prob:" << r.prob << " rect:" << r.rect << std::endl;
    }

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
