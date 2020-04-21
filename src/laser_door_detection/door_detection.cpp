#include "../../include/laser_door_detection/door_detection.h"
#include <algorithm>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/console.h"

namespace door_detection
{

DoorDetection::DoorDetection()
{
}

DoorDetection::~DoorDetection()
{
}

void DoorDetection::setLines(std::vector<line_extraction::Line> lines)
{
    lines_ = lines;
}

void DoorDetection::detectDoors(std::vector<Door> &doors)
{
    doors_.clear();
    
    filterLines();
    extractDoors();

    doors = doors_;
}

void DoorDetection::filterLines()
{
    std::vector<line_extraction::Line> tmp_lines;

    for (unsigned int i = 0; i < lines_.size(); i++)
    {
        // Filter small lines
        // TODO: set as parameter
        if (door_detection::euclideanDist(lines_[i].getStart(), lines_[i].getEnd()) > 0.5)
            tmp_lines.push_back(lines_[i]);
    }
    lines_ = tmp_lines;
}

void DoorDetection::extractDoors()
{
    for (unsigned int i = 0; i < lines_.size(); i++)
    {
        for (unsigned int j = 0; j < lines_.size(); j++)
        {
            // No doors w/ the same wall
            if (i == j)
                continue;

            Door door1(lines_[i].getStart(), lines_[j].getStart(), lines_[i], lines_[j]);
            Door door2(lines_[i].getStart(), lines_[j].getEnd(), lines_[i], lines_[j]);
            Door door3(lines_[i].getEnd(), lines_[j].getStart(), lines_[i], lines_[j]);
            Door door4(lines_[i].getEnd(), lines_[j].getEnd(), lines_[i], lines_[j]);
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

} // namespace door_detection
