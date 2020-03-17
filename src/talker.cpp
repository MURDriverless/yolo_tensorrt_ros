#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/package.h>

#include <sstream>

#include "class_detector.h"

#include <chrono>
#include <ctime>
#include <ratio>

// include the following for parsing and using the jetbot camera input
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



// image callback function to process received message
void imageCallback(const sensor_msgs::ImageConstPtr& msg, Detector &detector, std::vector<Result> &res)
{
  cv_bridge::CvImagePtr cv_ptr;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  // run inference with detector
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  {
    detector.detect(cv_ptr->image, res);
  }

  // can update GUI window here

  // output modified video stream
}

int main(int argc, char **argv)
{
  // TODO: rename 'talker' to more appropriate name
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

  std::vector<Result> res;

  // grab image from an image stream
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("jetbot_camera/raw", 1, 
    [&detector, &res](const sensor_msgs::ImageConstPtr& msg) -> void {imageCallback(msg, detector, res);});

  cv::Mat mat_image = cv::imread(img_path, cv::IMREAD_UNCHANGED);

  // auto t0 = std::chrono::high_resolution_clock::now();
  // detector.detect(mat_image, res);
  // auto t1 = std::chrono::high_resolution_clock::now();

  // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);

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
    // ss << "inference duration = " << time_span.count() << " seconds." << std::endl;

    // DEBUG
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
