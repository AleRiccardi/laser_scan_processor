#ifndef DOOR_DETECTION_DOOR_H
#define DOOR_DETECTION_DOOR_H

#include <boost/array.hpp>
#include <vector>
#include <memory>

#include "utilities.h"
#include "../laser_line_extraction/utilities.h"
#include "../laser_line_extraction/line.h"

namespace door_detection
{

class Door
{
public:
    Door(const boost::array<double, 2> start, const boost::array<double, 2> end,
         line_extraction::Line &line1, line_extraction::Line &line2);
    ~Door();

    double getAngle();
    double getWidth();
    const boost::array<double, 2> &getStart() const;
    const boost::array<double, 2> &getEnd() const;
    const line_extraction::Line &getLine1();
    const line_extraction::Line &getLine2();

    bool isInlier(boost::array<double, 2> point);

    // Override operator <
    bool operator<(const Door &door) const
    {
        return this->width_ < door.width_;
    }

private:
    // Line parameters
    double angle_;
    double width_;
    boost::array<double, 2> start_;
    boost::array<double, 2> end_;
    std::shared_ptr<line_extraction::Line> line1_;
    std::shared_ptr<line_extraction::Line> line2_;
};

} // namespace door_detection

#endif //DOOR_DETECTION_DOOR_H