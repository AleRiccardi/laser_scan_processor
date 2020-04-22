#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <cmath>
#include <vector>
#include <memory>
#include <boost/array.hpp>
#include <Eigen/Dense>
#include "utilities.h"
#include "line.h"
#include "../status.h"

namespace line_extraction
{

class LineExtraction
{

public:
  // Constructor / destructor
  LineExtraction(Status &status);
  ~LineExtraction();
  // Run
  void extractLines(std::vector<Line>&);
  // Parameter setting
  void setBearingVariance(double);
  void setRangeVariance(double);
  void setLeastSqAngleThresh(double);
  void setLeastSqRadiusThresh(double);
  void setMaxLineGap(double);
  void setMinLineLength(double);
  void setMinLinePoints(unsigned int);
  void setMinRange(double);
  void setMinSplitDist(double);
  void setOutlierDist(double);

private:
  // Data structures
  std::shared_ptr<CachedData> c_data_;
  std::shared_ptr<RangeData> r_data_;
  Params params_;
  // Indices after filtering
  std::vector<unsigned int> filtered_indices_;
  // Line data
  std::vector<Line> lines_;
  // Methods
  double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&,
                    const Eigen::Matrix2d&);
  double distBetweenPoints(unsigned int index_1, unsigned int index_2);
  void   filterClosePoints();
  void   filterOutlierPoints();
  void   filterLines();
  void   mergeLines();
  void   split(const std::vector<unsigned int>&);
};

} // namespace line_extraction

#endif
