#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include <math.h>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"

typedef struct
{
  double x;
  double y;
} Point;

class RangerFusion : public RangerFusionInterface
{
public:
  // Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();
  ~RangerFusion();

  // See rangerfusioninterface.h for more information
  // Accepts container of rangers - as per requirement C1
  void setRangers(std::vector<RangerInterface*> rangers);

  // Accepts container of cells - as per requirement C2
  void setCells(std::vector<Cell*> cells);

  // Grab data and fuse - as per requirement C3
  void grabAndFuseData();

  // Returns a container of raw data range readings - as per requirement C5
  std::vector<std::vector<double>> getRawRangeData();

private:
  void laser_check_(std::vector<double> data, RangerInterface* ranger);
  void sonarCheck_(std::vector<double> data, RangerInterface* cell);

  bool cone_intersection_(const Point& triangle_p1, const Point& traingle_p2, const Point& triangle_p3, const Point& test_p);
  bool line_intersection_(const Point& laser_point, const Point& laser_origin, const Point& cell_point_1,
                          const Point& cell_point_2);
  bool point_intersection_(const Point& point, Cell* cell);

  // This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_;
  std::vector<RangerInterface*> rangers_;
  std::vector<Cell*> cells_;

  const Point ORIGIN = { 0.0, 0.0 };
};

#endif  // RANGERFUSION_H
