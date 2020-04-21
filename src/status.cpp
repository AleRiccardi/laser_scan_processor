#include "../include/status.h"

Status::Status(/* args */)
{
}

Status::~Status()
{
}

void Status::setLines(std::vector<line_extraction::Line> &lines)
{
    lines_.clear();
    for (unsigned int i = 0; i < lines.size(); i++)
    {
        std::array<double, 4> line = {0};
        line[0] = lines[i].getStart()[0];
        line[1] = lines[i].getStart()[1];
        line[2] = lines[i].getEnd()[0];
        line[3] = lines[i].getEnd()[1];
        lines_.push_back(line);
    }
}

std::vector<std::array<double, 4>> Status::getLines()
{
    return lines_;
}
