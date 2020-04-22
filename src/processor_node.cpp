#include "../include/status.h"
#include "../include/laser_line_extraction/line_extraction_ros.h"
#include "../include/laser_door_detection/door_detection_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

  bool debug = false;
  double frequency;
  ros::init(argc, argv, "line_extraction_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  nh_local.param<bool>("debug", debug, false);
  if (debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_DEBUG("Starting line_extraction_node.");

  nh_local.param<double>("frequency", frequency, 25);
  ros::Rate rate(frequency);
  ROS_DEBUG("Frequency set to %0.1f Hz", frequency);

  Status status;
  line_extraction::LineExtractionROS line_extractor(nh, nh_local, status);
  door_detection::DoorDetectionROS door_detection(nh, nh_local, status);

  while (ros::ok())
  {
    line_extractor.run();
    door_detection.run();

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
