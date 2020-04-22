#ifndef STATUS_H
#define STATUS_H

#include <boost/array.hpp>
#include <vector>
#include <memory>

#include "laser_line_extraction/utilities.h"
#include "laser_line_extraction/line.h"
#include "laser_door_detection/door.h"

class Status
{
public:
    Status();
    ~Status();

    void setLines(std::vector<line_extraction::Line> &lines);
    void setDoors(std::vector<door_detection::Door> &doors);

    void setCachedData(const std::vector<double> &bearings,
                       const std::vector<double> &cos_bearings,
                       const std::vector<double> &sin_bearings,
                       const std::vector<unsigned int> &indices);
    void setRangeData(const std::vector<double> &ranges);

    std::shared_ptr<line_extraction::CachedData> getCachedData();
    std::shared_ptr<line_extraction::RangeData> getRangeData();
    std::shared_ptr<std::vector<line_extraction::Line>> getLines();
    std::shared_ptr<std::vector<door_detection::Door>> getDoors();
private:
    std::shared_ptr<line_extraction::CachedData> c_data_;
    std::shared_ptr<line_extraction::RangeData> r_data_;
    std::shared_ptr<std::vector<line_extraction::Line>> lines_;
    std::shared_ptr<std::vector<door_detection::Door>> doors_;
};

#endif //STATUS_H