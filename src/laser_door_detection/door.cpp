#include "../../include/laser_door_detection/door.h"

namespace door_detection
{

Door::Door(const boost::array<double, 2> start, const boost::array<double, 2> end, line_extraction::Line &line1, line_extraction::Line &line2)
{
    start_ = start;
    end_ = end;
    line1_ = std::make_shared<line_extraction::Line>(line1);
    line2_ = std::make_shared<line_extraction::Line>(line2);
    width_ = door_detection::euclideanDist(start_, end_);
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

} // namespace door_detection
