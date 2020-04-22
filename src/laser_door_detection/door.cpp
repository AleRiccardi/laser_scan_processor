#include "../../include/laser_door_detection/door.h"
#include "ros/console.h"

namespace door_detection
{

Door::Door(const boost::array<double, 2> start, const boost::array<double, 2> end, line_extraction::Line &line1, line_extraction::Line &line2)
{
  start_ = start;
  end_ = end;
  line1_ = std::make_shared<line_extraction::Line>(line1);
  line2_ = std::make_shared<line_extraction::Line>(line2);
  width_ = door_detection::euclideanDist(start_, end_);
  angle_ = angleFromEndpoints(start_, end_);
}

Door::~Door()
{
  line1_.reset();
  line2_.reset();
}

double Door::getAngle()
{
  return angle_;
}
double Door::getWidth()
{
  return width_;
}

const boost::array<double, 2> &Door::getStart() const
{
  return start_;
}

const boost::array<double, 2> &Door::getEnd() const
{
  return end_;
}

const double dotPoints(boost::array<double, 2> x1, boost::array<double, 2> x2)
{
  return x1[0] * x2[0] + x1[1] * x2[1];
}

bool Door::isPointInline(boost::array<double, 2> point, double threshold)
{
  // Return minimum distance between line segment vw and point p
  const double l2 = pow(start_[0] - end_[0], 2) + pow(start_[1] - end_[1], 2);

  if (l2 == 0.0)
    return door_detection::euclideanDist(point, start_); // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line.
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  boost::array<double, 2> sub1 = {point[0] - start_[0], point[1] - start_[1]};
  boost::array<double, 2> sub2 = {end_[0] - start_[0], end_[1] - start_[1]};
  float t = std::max(0.0, std::min(1.0, dotPoints(sub1, sub2) / l2));
  // Projection on the segment
  boost::array<double, 2> pre_proj = {t * sub2[0], t * sub2[1]};
  boost::array<double, 2> proj = {start_[0] + pre_proj[0], start_[1] + pre_proj[1]};

  if (euclideanDist(point, proj) < threshold)
  {
    double ang_proj = angleFromEndpoints(point, proj);
    double ang_diff = fabs(ang_proj - angle_);
    double right_angle = ang_diff * (180 / M_PI);
    bool isInline(89 < right_angle && right_angle < 91);
    ROS_DEBUG("%f, %f, %s", ang_proj, angle_, (isInline ? "true" : "false"));
    ROS_DEBUG("line proj: (%f, %f), (%f, %f)",point[0], proj[0], point[1], proj[1]);
    ROS_DEBUG("line door: (%f, %f), (%f, %f)",start_[0], end_[0], start_[1], end_[1]);
    return isInline;
  }
  return false;
}

} // namespace door_detection
