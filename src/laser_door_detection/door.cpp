#include "../../include/laser_door_detection/door.h"
#include "ros/console.h"

namespace door_detection
{

/**
 * Door constructour.
 */
Door::Door(const boost::array<double, 2> start, const boost::array<double, 2> end,
           line_extraction::Line &line1, line_extraction::Line &line2)
{
  start_ = start;
  end_ = end;
  line1_ = std::make_shared<line_extraction::Line>(line1);
  line2_ = std::make_shared<line_extraction::Line>(line2);
  width_ = door_detection::euclideanDist(start_, end_);
  angle_ = angleFromEndpoints(start_, end_);
}

/**
 * Door destroctour.
 */
Door::~Door()
{
  line1_.reset();
  line2_.reset();
}

/**
 * Get angle.
 */
double Door::getAngle()
{
  return angle_;
}

/**
 * Get width.
 */
double Door::getWidth()
{
  return width_;
}

/**
 * Get door starting point.
 */
const boost::array<double, 2> &Door::getStart() const
{
  return start_;
}

/**
 * Get door ending point.
 */
const boost::array<double, 2> &Door::getEnd() const
{
  return end_;
}

/**
 * Get line1 that generated the door.
 */
const line_extraction::Line &Door::getLine1()
{
  return *line1_;
}

/**
 * Get line2 that generated the door.
 */
const line_extraction::Line &Door::getLine2()
{
  return *line2_;
}

/**
 * Dot product between two point.
 */
const double dotPoints(boost::array<double, 2> x1, boost::array<double, 2> x2)
{
  return x1[0] * x2[0] + x1[1] * x2[1];
}

/**
 * Understand if a given point is an inlier of the door.
 * It is used a Gaussian function in order to evaluate if 
 * the distance between the door and the point is allowed.
 * The allowed distance gets bigger in the center and smaller
 * at the border of the door.
 */
bool Door::isInlier(boost::array<double, 2> point)
{
  // Return minimum distance between line segment vw and point p
  const double l2 = pow(start_[0] - end_[0], 2) + pow(start_[1] - end_[1], 2);
  boost::array<double, 2> sub1 = {point[0] - start_[0], point[1] - start_[1]};
  boost::array<double, 2> sub2 = {end_[0] - start_[0], end_[1] - start_[1]};
  float t = dotPoints(sub1, sub2) / l2;
  // Extract projection into the line (door)
  boost::array<double, 2> pre_proj = {t * sub2[0], t * sub2[1]};
  boost::array<double, 2> proj = {start_[0] + pre_proj[0], start_[1] + pre_proj[1]};

  // Check if the projection is
  // between the door's margins.
  if (0 <= t && t <= 1)
  {
    // Compute the position of the projection
    // in the line between 0 and 1.
    double pos1 = (proj[0] - end_[0]) / (start_[0] - end_[0]);

    // A Gaussian function extract the right threshold
    // to evaluate if the distance between the point and
    // the line is reasonable.
    double max_dist = 0.4;
    double mean = 0.5;
    double sigma = 0.15;
    double threshold = max_dist * exp(-0.5 * pow(((pos1 - mean) / sigma), 2));
    return euclideanDist(point, proj) < threshold;
  }
  return false;
}

} // namespace door_detection
