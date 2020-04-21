#include "../include/status.h"

Status::Status(/* args */)
{
}

Status::~Status()
{
}

void Status::setLines(std::vector<line_extraction::Line> &lines)
{
    lines_ = lines;
}

std::vector<line_extraction::Line> Status::getLines()
{
    return lines_;
}
