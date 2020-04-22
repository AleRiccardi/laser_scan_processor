#include "../../include/laser_door_detection/door_detection_ros.h"

namespace door_detection
{

/**
 * DoorDetectionROS constroctour.
 */
DoorDetectionROS::DoorDetectionROS(ros::NodeHandle &nh, ros::NodeHandle &nh_local, Status &status)
    : nh_(nh), nh_local_(nh_local), door_detection_(status)
{
  status_ = std::make_shared<Status>(status);

  loadParameters();
  line_door_pub_ = nh_.advertise<laser_scan_processor::LineSegmentList>("door_segments", 1);
  if (pub_markers_door_)
  {
    marker_door_pub_ = nh_.advertise<visualization_msgs::Marker>("line_door_markers", 1);
  }
}

/**
 * DoorDetectionROS destroctour.
 */
DoorDetectionROS::~DoorDetectionROS()
{
  status_.reset();
}

/**
 * Run function.
 */
void DoorDetectionROS::run()
{
  std::vector<Door> doors;
  door_detection_.detectDoors(doors);

  status_->setDoors(doors);
  // Also publish markers if parameter publish_markers is set to true
  if (pub_markers_door_)
  {
    visualization_msgs::Marker marker_msg;
    populateMarkerMsg(doors, marker_msg);
    marker_door_pub_.publish(marker_msg);
  }
}

/**
 * Load ROS parameters.
 */
void DoorDetectionROS::loadParameters()
{

  ROS_DEBUG("*************************************");
  ROS_DEBUG("DOOR PARAMETERS:");

  // Parameters used by this node

  std::string frame_door_id, scan_topic;
  bool pub_markers_door;
  double min_line_door_length;
  double min_door_width;
  double max_door_width;
  int max_allowed_inliers;
  double min_allowed_angle;
  double max_allowed_angle;

  nh_local_.param<std::string>("frame_door_id", frame_door_id, "laser");
  frame_door_id_ = frame_door_id;
  ROS_DEBUG("frame_id: %s", frame_door_id_.c_str());

  nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
  scan_topic_ = scan_topic;
  ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

  nh_local_.param<bool>("publish_markers", pub_markers_door, true);
  pub_markers_door_ = pub_markers_door;
  ROS_DEBUG("publish_markers: %s", pub_markers_door_ ? "true" : "false");

  nh_local_.param<double>("min_line_door_length", min_line_door_length, 0.8);
  door_detection_.setMinLineDoorLength(min_line_door_length);
  ROS_DEBUG("min_line_door_length: %f", min_line_door_length);

  nh_local_.param<double>("min_door_width", min_door_width, 0.7);
  door_detection_.setMinDoorWidth(min_door_width);
  ROS_DEBUG("min_door_width: %f", min_door_width);

  nh_local_.param<double>("max_door_width", max_door_width, 1.05);
  door_detection_.setMaxDoorWidth(max_door_width);
  ROS_DEBUG("max_door_width: %f", max_door_width);

  nh_local_.param<int>("max_allowed_inliers", max_allowed_inliers, 3);
  door_detection_.setMaxAllowedInliers(max_allowed_inliers);
  ROS_DEBUG("max_allowed_inliers: %d", max_allowed_inliers);

  nh_local_.param<double>("min_allowed_angle", min_allowed_angle, 40);
  door_detection_.setMinAllowedAngle(min_allowed_angle);
  ROS_DEBUG("min_allowed_angle: %f", min_allowed_angle);

  nh_local_.param<double>("max_allowed_angle", max_allowed_angle, 200);
  door_detection_.setMaxAllowedAngle(max_allowed_angle);
  ROS_DEBUG("max_allowed_angle: %f", max_allowed_angle);

  ROS_DEBUG("*************************************");
}

/**
 * Populate a marker message with all the doors.
 */
void DoorDetectionROS::populateMarkerMsg(const std::vector<Door> &doors,
                                         visualization_msgs::Marker &marker_msg)
{
  marker_msg.ns = "door_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.05;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 1.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Door>::const_iterator cit = doors.begin(); cit != doors.end(); ++cit)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_door_id_;
  marker_msg.header.stamp = ros::Time::now();
}

} // namespace door_detection
