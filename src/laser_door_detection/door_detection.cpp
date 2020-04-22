#include "../../include/laser_door_detection/door_detection.h"
#include <algorithm>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/console.h"

namespace door_detection
{

DoorDetection::DoorDetection(Status &status)
{
    c_data_ = status.getCachedData();
    r_data_ = status.getRangeData();
    lines_ = status.getLines();
}

DoorDetection::~DoorDetection()
{
}

void DoorDetection::detectDoors(std::vector<Door> &doors)
{
    doors_.clear();

    extractDoors();
    filterDoorsWithInliners();
    filterDoorsWrongAngleLines();

    doors = doors_;
}

std::vector<line_extraction::Line> DoorDetection::filterLines()
{
    std::vector<line_extraction::Line> tmp_lines;

    for (unsigned int i = 0; i < lines_->size(); i++)
    {
        // Filter small lines
        // TODO: set as parameter
        if (door_detection::euclideanDist(lines_->at(i).getStart(), lines_->at(i).getEnd()) > 0.5)
            tmp_lines.push_back(lines_->at(i));
    }
    return tmp_lines;
}

void DoorDetection::extractDoors()
{
    std::vector<line_extraction::Line> filtered_lines = filterLines();

    for (unsigned int i = 0; i < filtered_lines.size(); i++)
    {
        for (unsigned int j = 0; j < lines_->size(); j++)
        {
            // No doors w/ the same wall
            if (i == j)
                continue;

            Door door1(filtered_lines[i].getStart(), filtered_lines[j].getStart(), filtered_lines[i], filtered_lines[j]);
            Door door2(filtered_lines[i].getStart(), filtered_lines[j].getEnd(), filtered_lines[i], filtered_lines[j]);
            Door door3(filtered_lines[i].getEnd(), filtered_lines[j].getStart(), filtered_lines[i], filtered_lines[j]);
            Door door4(filtered_lines[i].getEnd(), filtered_lines[j].getEnd(), filtered_lines[i], filtered_lines[j]);
            std::vector<Door> doors = {door1, door2, door3, door4};

            // Filter Doors from distances
            // ----------------------------
            int count = 0;
            boost::array<boost::array<double, 2>, 2> close_points;
            std::sort(doors.begin(), doors.end());

            for (unsigned int k = 0; k < doors.size(); k++)
            {
                // Max num of doors given by two walls < 2
                if (count >= 2)
                    break;

                // Check Not Possible Doors
                // ------------------------
                if (close_points[0] == doors[k].getStart() || close_points[1] == doors[k].getEnd())
                    continue;

                // Detect Not Possible Doors
                // -------------------------
                // If the distance between two extremities of two lines is minor of a threshold,
                // then there is no chance to have any door between one of these two points.
                if (doors[k].getWidth() < 0.8)
                {
                    // saving the walls' points
                    close_points[0] = doors[k].getStart();
                    close_points[1] = doors[k].getEnd();
                }

                // Possible Door Detected
                // -----------------------
                // TODO: Set param for thresholds
                if (0.8 < doors[k].getWidth() && doors[k].getWidth() < 1)
                {
                    doors_.push_back(doors[k]);
                    count++;
                }
            }
        }
    }
}

void DoorDetection::filterDoorsWithInliners()
{
    return;
}

void DoorDetection::filterDoorsWrongAngleLines()
{
    return;
}

} // namespace door_detection
