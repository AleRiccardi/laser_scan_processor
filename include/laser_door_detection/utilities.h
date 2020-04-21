#ifndef DOOR_DETECTION_UTILITIES_H
#define DOOR_DETECTION_UTILITIES_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <math.h>
#include <boost/array.hpp>

namespace door_detection
{


inline double euclideanDist(boost::array<double, 2> p1, boost::array<double, 2> p2)
{
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

} // namespace door_detection

#endif
