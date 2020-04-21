#ifndef STATUS_H
#define STATUS_H

#include <array>
#include <vector>
#include "laser_line_extraction/line.h"

class Status
{
public:
    Status();
    ~Status();

    void setLines(std::vector<line_extraction::Line> &lines);
    std::vector<std::array<double, 4>> getLines();
    
private:
      std::vector<std::array<double, 4>> lines_;
};


#endif //STATUS_H