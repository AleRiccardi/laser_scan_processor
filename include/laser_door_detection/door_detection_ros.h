#ifndef DOOR_DETECTION_ROS_H
#define DOOR_DETECTION_ROS_H

#include <iostream>
#include <vector>
#include <boost/array.hpp>
#include <string>
#include <cmath>
#include <bits/floatn-common.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ros/console.h>

#include "../status.h"
#include "door.h"
#include "door_detection.h"
#include "laser_scan_processor/LineSegment.h"
#include "laser_scan_processor/LineSegmentList.h"

namespace door_detection
{

class DoorDetectionROS
{

public:
  // Constructor / destructor
  DoorDetectionROS(ros::NodeHandle &, ros::NodeHandle &, Status &status);
  ~DoorDetectionROS();
  // Running
  void run();
  void loadParameters();

private:
  void lineWallCallback(const laser_scan_processor::LineSegmentList::ConstPtr &scan_msg);
  void populateMarkerMsg(const std::vector<Door> &doors,
                         visualization_msgs::Marker &marker_msg);

  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::Publisher line_door_pub_;
  ros::Publisher marker_door_pub_;

  std::shared_ptr<Status> status_;
  // Parameters
  std::string frame_door_id_;
  std::string scan_topic_;
  bool pub_markers_door_;
  // Line extraction
  DoorDetection door_detection_;
};

} // namespace door_detection

#endif
