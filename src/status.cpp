#include "../include/status.h"
#include "ros/console.h"

Status::Status(/* args */)
    : c_data_(new line_extraction::CachedData()),
      r_data_(new line_extraction::RangeData()),
      lines_(new std::vector<line_extraction::Line>()),
      doors_(new std::vector<door_detection::Door>())
{
}

Status::~Status()
{
}

void Status::setLines(std::vector<line_extraction::Line> &lines)
{
    lines_->clear();
    for (unsigned int i = 0; i < lines.size(); i++)
    {
        lines_->push_back(lines[i]);
    }
}

void Status::setDoors(std::vector<door_detection::Door> &doors)
{
    doors_->clear();
    for (unsigned int i = 0; i < doors.size(); i++)
    {
        doors_->push_back(doors[i]);
    }
}

void Status::setCachedData(const std::vector<double> &bearings,
                           const std::vector<double> &cos_bearings,
                           const std::vector<double> &sin_bearings,
                           const std::vector<unsigned int> &indices)
{
    c_data_->bearings = bearings;
    c_data_->cos_bearings = cos_bearings;
    c_data_->sin_bearings = sin_bearings;
    c_data_->indices = indices;
}

void Status::setRangeData(const std::vector<double> &ranges)
{
    r_data_->ranges = ranges;
    r_data_->xs.clear();
    r_data_->ys.clear();
    for (std::vector<unsigned int>::const_iterator cit = c_data_->indices.begin();
         cit != c_data_->indices.end(); ++cit)
    {
        r_data_->xs.push_back(c_data_->cos_bearings[*cit] * ranges[*cit]);
        r_data_->ys.push_back(c_data_->sin_bearings[*cit] * ranges[*cit]);
    }
}

std::shared_ptr<line_extraction::CachedData> Status::getCachedData()
{
    return c_data_;
}
std::shared_ptr<line_extraction::RangeData> Status::getRangeData()
{
    return r_data_;
}

std::shared_ptr<std::vector<line_extraction::Line>> Status::getLines()
{
    return lines_;
}

std::shared_ptr<std::vector<door_detection::Door>> Status::getDoors()
{
    return doors_;
}
