#ifndef STATUS_H
#define STATUS_H

#include <boost/array.hpp>
#include <vector>
#include "laser_line_extraction/line.h"

class Status
{
public:
    Status();
    ~Status();

    void setLines(std::vector<line_extraction::Line> &lines);
    std::vector<line_extraction::Line> getLines();
    
private:
      std::vector<line_extraction::Line> lines_;
};


#endif //STATUS_H