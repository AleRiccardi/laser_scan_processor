#include "../../include/laser_door_detection/door_detection.h"
#include <algorithm>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/console.h"

std::vector<int> sort2idx(std::vector<double> vect)
{
    std::size_t n(0);
    std::vector<int> idx(vect.size());
    std::generate(std::begin(idx), std::end(idx), [&] { return n++; });
    std::sort(std::begin(idx),
              std::end(idx),
              [&](int i1, int i2) { return vect[i1] < vect[i2]; });
    return idx;
}

namespace door_detection
{

DoorDetection::DoorDetection()
{
}

DoorDetection::~DoorDetection()
{
}

void DoorDetection::setLines(std::vector<std::array<double, 4>> lines)
{
    lines_ = lines;
}

void DoorDetection::detectDoors(std::vector<std::array<double, 4>> &doors)
{
    filterLines();

    doors_.clear();
    for (unsigned int i = 0; i < lines_.size(); i++)
    {
        for (unsigned int j = 0; j < lines_.size(); j++)
        {
            // No doors w/ the same wall
            if (i == j)
                continue;

            std::vector<double> dists;
            std::vector<std::array<double, 4>> tmp_doors;
            std::vector<std::array<std::array<double, 4>, 2>> walls_of_doors;

            for (unsigned k = 0; k < 4; k += 2)
            {
                for (unsigned l = 0; l < 4; l += 2)
                {
                    double dist = euclideanDist(lines_[i][k], lines_[j][l], lines_[i][k + 1], lines_[j][l + 1]);

                    dists.push_back(dist);
                    tmp_doors.push_back({lines_[i][k], lines_[i][k + 1], lines_[j][l], lines_[j][l + 1]});
                    walls_of_doors.push_back({lines_[i], lines_[j]});
                }
            }

            int count = 0;
            std::array<double, 4> no_door_points = {-1};
            std::vector<int> idx = sort2idx(dists);
            //std::vector<int> idx = {0,1,2,3};
            for (unsigned int k = 0; k < idx.size(); k++)
            {
                // Max num of doors given by two walls < 2
                if (count >= 2)
                    break;

                // Check not possible doors
                if (no_door_points[0] != -1)
                {
                    bool first = (no_door_points[0] == tmp_doors[idx[k]][0] && no_door_points[1] == tmp_doors[idx[k]][1]);
                    bool second = (no_door_points[2] == tmp_doors[idx[k]][2] && no_door_points[3] == tmp_doors[idx[k]][3]);
                    if (first || second)
                        continue;
                }

                // If the distance between two extremities of two walls is less than a threshold, then
                // there is no chance to have any door between one of these two points of the walls.
                if (dists[idx[k]] < 0.8)
                    // saving the walls' points
                    no_door_points = tmp_doors[idx[k]];

                // Possible door detected
                if (0.8 < dists[idx[k]] && dists[idx[k]] < 1)
                {
                    doors_.push_back(tmp_doors[idx[k]]);
                    count++;
                }
            }
        }
    }
    doors = doors_;
} // namespace door_detection

void DoorDetection::filterLines()
{
    std::vector<std::array<double, 4>> tmp_lines;

    for (unsigned int i = 0; i < lines_.size(); i++)
    {
        double dist = sqrt(pow(lines_[i][0] - lines_[i][2], 2) + pow(lines_[i][1] - lines_[i][3], 2));

        // Filter small lines
        // TODO: set as parameter
        if (dist > 0.5)
            tmp_lines.push_back(lines_[i]);
    }
    lines_ = tmp_lines;
}

double DoorDetection::euclideanDist(double a, double b, double c, double d)
{
    return sqrt(pow(a - b, 2) + pow(c - d, 2));
}

} // namespace door_detection
