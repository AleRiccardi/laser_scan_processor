#ifndef DOOR_DETECTION_H
#define DOOR_DETECTION_H

#include <vector>
#include <boost/array.hpp>

#include "../status.h"
#include "door.h"
#include "utilities.h"
#include "../laser_line_extraction/utilities.h"
#include "../laser_line_extraction/line.h"
#include "laser_scan_processor/LineSegment.h"

namespace door_detection
{

class DoorDetection
{
public:
    DoorDetection(Status &status);
    ~DoorDetection();

    void detectDoors(std::vector<Door> &doors);

private:
    std::vector<line_extraction::Line> filterLines();
    void extractDoors();
    void filterDoorsWithInliners();
    void filterDoorsWrongAngleLines();

    std::shared_ptr<line_extraction::CachedData> c_data_;
    std::shared_ptr<line_extraction::RangeData> r_data_;
    std::shared_ptr<std::vector<line_extraction::Line>> lines_;
    std::shared_ptr<line_extraction::Params> params_line_;

    std::vector<Door> doors_;
};

} // namespace door_detection

#endif