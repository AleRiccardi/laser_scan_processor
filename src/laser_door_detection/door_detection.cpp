#include "../../include/laser_door_detection/door_detection.h"
#include <algorithm>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/console.h"

namespace door_detection
{

/**
 * DoorDetection constroctour.
 */
DoorDetection::DoorDetection(Status &status)
{
    c_data_ = status.getCachedData();
    r_data_ = status.getRangeData();
    lines_ = status.getLines();
    params_line_ = status.getParamsLine();
}

/**
 * DoorDetection destructour.
 */
DoorDetection::~DoorDetection()
{
    c_data_.reset();
    r_data_.reset();
    lines_.reset();
    params_line_.reset();
}

/**
 * Main function called for door detection.
 */
void DoorDetection::detectDoors(std::vector<Door> &doors)
{
    doors_.clear();

    extractDoors();
    filterDoorsInliers();
    filterDoorsAngles();

    doors = doors_;
}

/**
 * Filter out lines not allows lines.
 */
std::vector<line_extraction::Line> DoorDetection::filterLines()
{
    std::vector<line_extraction::Line> tmp_lines;

    for (unsigned int i = 0; i < lines_->size(); i++)
    {
        // Filter small lines
        if (door_detection::euclideanDist(lines_->at(i).getStart(), lines_->at(i).getEnd()) > params_.min_line_door_length)
            tmp_lines.push_back(lines_->at(i));
    }
    return tmp_lines;
}

/**
 * Extract doors from lines.
 */
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

            // Create all possible door combinations given two walls
            Door door1(filtered_lines[i].getStart(), filtered_lines[j].getStart(), filtered_lines[i], filtered_lines[j]);
            Door door2(filtered_lines[i].getStart(), filtered_lines[j].getEnd(), filtered_lines[i], filtered_lines[j]);
            Door door3(filtered_lines[i].getEnd(), filtered_lines[j].getStart(), filtered_lines[i], filtered_lines[j]);
            Door door4(filtered_lines[i].getEnd(), filtered_lines[j].getEnd(), filtered_lines[i], filtered_lines[j]);
            std::vector<Door> doors = {door1, door2, door3, door4};

            int count = 0;
            boost::array<boost::array<double, 2>, 2> close_points;
            // Sort doors by their width
            std::sort(doors.begin(), doors.end());

            // Filter Doors by width
            // ---------------------
            for (unsigned int k = 0; k < doors.size(); k++)
            {
                // No more than two doors
                // between two walls.
                if (count >= 2)
                    break;

                // Check Not Possible Doors
                if (close_points[0] == doors[k].getStart() || close_points[1] == doors[k].getEnd())
                    continue;

                // Detect Not Possible Doors
                // If the distance between two extremities of two lines is minor of a threshold,
                // then there is no chance to have any door between one of these two points.
                if (doors[k].getWidth() < 0.8)
                {
                    // saving the walls' points
                    close_points[0] = doors[k].getStart();
                    close_points[1] = doors[k].getEnd();
                }

                // Possible Door Detected
                // TODO: Set param for thresholds
                if (params_.min_door_width < doors[k].getWidth() && doors[k].getWidth() < params_.max_door_width)
                {
                    doors_.push_back(doors[k]);
                    count++;
                }
            }
        }
    }
}

/**
 * Filter out doors that have inliers points and keep doors
 * with a gap between the walls. This will help to remove 
 * fake doors detected along walls.
 */
void DoorDetection::filterDoorsInliers()
{
    std::vector<Door> tmp_doors;
    for (unsigned int i = 0; i < doors_.size(); i++)
    {
        unsigned int count = 0;
        for (std::vector<unsigned int>::const_iterator cit = c_data_->indices.begin();
             cit != c_data_->indices.end(); ++cit)
        {
            boost::array<double, 2> point = {r_data_->xs[*cit], r_data_->ys[*cit]};
            // Check if the bearing point is inlier the door
            if (doors_[i].isInlier(point))
                count++;
        }

        // Filter out the door if more than
        // two inline point were detected.
        if (count < params_.max_allowed_inliers)
            tmp_doors.push_back(doors_[i]);
    }
    doors_ = tmp_doors;
}

/**
 * Filter out doors that are generated with a 
 * not allowed angle between the two walls.
 */
void DoorDetection::filterDoorsAngles()
{
    std::vector<Door> tmp_doors;
    for (unsigned int i = 0; i < doors_.size(); i++)
    {
        // Get the angle of the first and second wall that generated the door
        double ang1 = angleFromEndpoints(doors_[i].getLine1().getStart(), doors_[i].getLine1().getEnd());
        double ang2 = angleFromEndpoints(doors_[i].getLine2().getStart(), doors_[i].getLine2().getEnd());
        // Compute the angles diff and convert to degree
        double ang_diff = (ang1 - ang2) * (180 / M_PI);

        // Filter out doors with wrong walls angle
        if (params_.min_allowed_angle <= ang_diff && ang_diff <= params_.max_allowed_angle)
            tmp_doors.push_back(doors_[i]);
    }

    doors_ = tmp_doors;
}

/**
 * Set Parameters.
 * ---------------
 */
void DoorDetection::setMinLineDoorLength(double value)
{
    params_.min_line_door_length = value;
}
void DoorDetection::setMinDoorWidth(double value)
{
    params_.min_door_width = value;
}
void DoorDetection::setMaxDoorWidth(double value)
{
    params_.max_door_width = value;
}
void DoorDetection::setMaxAllowedInliers(int value)
{
    params_.max_allowed_inliers = value;
}
void DoorDetection::setMinAllowedAngle(double value)
{
    params_.min_allowed_angle = value;
}
void DoorDetection::setMaxAllowedAngle(double value)
{
    params_.max_allowed_angle = value;
}

} // namespace door_detection
