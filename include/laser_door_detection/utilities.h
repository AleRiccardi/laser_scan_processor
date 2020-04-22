#ifndef DOOR_DETECTION_UTILITIES_H
#define DOOR_DETECTION_UTILITIES_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <math.h>
#include <boost/array.hpp>

#include "../laser_line_extraction/utilities.h"

namespace door_detection
{

struct Params
{
  double min_line_door_length;
  double min_door_width;
  double max_door_width;
  int max_allowed_inliers;
  double min_allowed_angle;
  double max_allowed_angle;
};


/**
 * Compute the euclidean distance between two points.
 */
inline double euclideanDist(boost::array<double, 2> p1, boost::array<double, 2> p2)
{
  return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

/**
 * Compute angle between two endpoints.
 */
inline double angleFromEndpoints(boost::array<double, 2> p1, boost::array<double, 2> p2)
{
  double slope, angle;
  if (fabs(p2[0] - p1[0]) > 1e-9)
  {
    slope = (p2[1] - p1[1]) / (p2[0] - p1[0]);
    return line_extraction::pi_to_pi(atan(slope) + M_PI / 2);
  }

  return 0.0;
}

} // namespace door_detection

#endif
