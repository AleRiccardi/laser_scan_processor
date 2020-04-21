#ifndef DOOR_DETECTION_H
#define DOOR_DETECTION_H

#include <vector>
#include <array>

#include "laser_scan_processor/LineSegment.h"

namespace door_detection
{

class DoorDetection
{
public:
    DoorDetection();
    ~DoorDetection();

    void setLines(std::vector<std::array<double, 4>> lines);
    void detectDoors(std::vector<std::array<double, 4>> &doors);

private:
    void filterLines();
    double euclideanDist(double a, double b, double c, double d);
    // lines [[x,y,x,y], ...]
    std::vector<std::array<double, 4>> lines_;
    std::vector<std::array<double, 4>> doors_;
};

} // namespace door_detection

#endif