#ifndef DOOR_DETECTION_H
#define DOOR_DETECTION_H

#include <vector>
#include <boost/array.hpp>

#include "door.h"
#include "utilities.h"
#include "../laser_line_extraction/line.h"
#include "laser_scan_processor/LineSegment.h"

namespace door_detection
{

class DoorDetection
{
public:
    DoorDetection();
    ~DoorDetection();

    void setLines(std::vector<line_extraction::Line> lines);
    void detectDoors(std::vector<Door> &doors);

private:
    void filterLines();
    void extractDoors();
    
    std::vector<line_extraction::Line> lines_;
    std::vector<Door> doors_;
};

} // namespace door_detection

#endif