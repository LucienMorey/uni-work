#include "rangerfusion.h"

#include <iostream>

RangerFusion::RangerFusion()
{
}

RangerFusion::~RangerFusion()
{
}

void RangerFusion::setRangers(std::vector<RangerInterface*> rangers)
{
  rangers_ = rangers;
}

void RangerFusion::setCells(std::vector<Cell*> cells)
{
  cells_ = cells;
}

void RangerFusion::grabAndFuseData()
{
  getRawRangeData();
  for (auto cell : cells_)
  {
    cell->setState(UNKNOWN);
  }
  auto data_it = data_.begin();
  auto ranger_it = rangers_.begin();
  for (; ((data_it < data_.end()) && (ranger_it < rangers_.end())); ++ranger_it, ++data_it)
  {
    if ((*ranger_it)->getSensingMethod() == POINT)
    {
      laser_check_((*data_it), (*ranger_it));
    }
    else
    {
      // sonar_check_(data,cells_);
    }
  }
}

std::vector<std::vector<double>> RangerFusion::getRawRangeData()
{
  data_.clear();
  for (auto ranger : rangers_)
  {
    data_.push_back(ranger->generateData());
  }
  return data_;
}

void RangerFusion::laser_check_(std::vector<double> data, RangerInterface* ranger)
{
  for (auto cell : cells_)
  {
    int laser_angle = 180 + ranger->getOffset();
    double mid_x, mid_y;
    cell->getCentre(mid_x, mid_y);

    Point cell_top_left = { (mid_x - cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_top_right = { (mid_x + cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_bottom_left = { (mid_x - cell->getSide() / 2), (mid_y - cell->getSide() / 2) };
    Point cell_bottom_right = { (mid_x + cell->getSide() / 2), (mid_y - cell->getSide() / 2) };

    for (auto data_point : data)
    {
      if (cell->getState() == OCCUPIED)
      {
        laser_angle -= ranger->getAngularResolution();
        break;
      }

      Point laser_point = { data_point * cos((M_PI / 180) * laser_angle),
                            data_point * sin((M_PI / 180) * laser_angle) };
      Point laser_origin = { ranger->getMinRange() * cos((M_PI / 180) * laser_angle),
                             ranger->getMinRange() * cos((M_PI / 180) * laser_angle) };
      laser_angle -= ranger->getAngularResolution();

      if (point_intersection_(laser_point, cell))
      {
        cell->setState(OCCUPIED);
        break;
      }
      else if (((laser_point.x != laser_origin.x) && (laser_point.y != laser_origin.y)) &&
               ((line_intersection_(laser_point, laser_origin, cell_top_left, cell_bottom_right)) ||
                (line_intersection_(laser_point, laser_origin, cell_bottom_left, cell_top_right))))
      {
        cell->setState(FREE);
      }
    }
    std::cout << std::endl;
  }
}

void RangerFusion::sonarCheck_(std::vector<double> data, RangerInterface* ranger)
{
  for (auto cell : cells_)
  {
    double sonar_angle = 90 + ranger->getOffset();
    double mid_x, mid_y;
    cell->getCentre(mid_x, mid_y);

    Point cell_top_left = { (mid_x - cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_top_right = { (mid_x + cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_bottom_left = { (mid_x - cell->getSide() / 2), (mid_y - cell->getSide() / 2) };
    Point cell_bottom_right = { (mid_x + cell->getSide() / 2), (mid_y - cell->getSide() / 2) };
    for (auto data_point : data)
    {
      if (cell->getState() == OCCUPIED)
      {
        break;
      }

      Point sonar_origin = { ranger->getMinRange() * cos((M_PI / 180) * sonar_angle),
                             ranger->getMinRange() * cos((M_PI / 180) * sonar_angle) };

      Point sonar_point_1 = { data_point * sin((M_PI / 180) * (sonar_angle + ranger->getAngularResolution())),
                              data_point * cos((M_PI / 180) * (sonar_angle + ranger->getAngularResolution())) };

      Point sonar_point_2 = { data_point * sin((M_PI / 180) * (sonar_angle - ranger->getAngularResolution())),
                              data_point * cos((M_PI / 180) * (sonar_angle - ranger->getAngularResolution())) };

      if (line_intersection_(sonar_point_1, sonar_point_2, cell_bottom_right, cell_top_left) ||
          line_intersection_(sonar_point_1, sonar_point_2, cell_top_right, cell_bottom_left) ||
          point_intersection_(sonar_point_1, cell) || point_intersection_(sonar_point_2, cell))
      {
        cell->setState(OCCUPIED);
      }
    }
  }
}

/**
 * @brief test if point intercepts a cell (rectangle)
 * 
 * @param point point to be tested
 * @param cell cell to be tested
 * @return true if point lies within or on boundary of cell
 * @return false if point is ourside of cell
 */
bool RangerFusion::point_intersection_(const Point& point, Cell* cell)
{
  //obtain midpoint to easily find vertexes of cell
  double mid_x, mid_y;
  cell->getCentre(mid_x, mid_y);

  //boundaries are found with midpoint +- the cell side length
  //returns true if point intersects with cell
  return ((point.x <= (mid_x + (cell->getSide() / 2))) && (point.x >= (mid_x - (cell->getSide() / 2))) &&
          (point.y <= (mid_y + (cell->getSide() / 2))) && (point.y >= (mid_y - (cell->getSide() / 2))));
}

/**
 * @brief check if two line segments intercept each other
 * 
 * @param p1 line segment 1 point 1
 * @param p2 line segment 1 point 2
 * @param p3 line segment 2 point 1
 * @param p4 line segment 2 point 2
 * @return true if interception occurs within range of both line segments
 * @return false if interception doesnt occur between the range of both line segments
 */
bool RangerFusion::line_intersection_(const Point& p1, const Point& p2, const Point& p3, const Point& p4)
{
  // credit to Paulborke for algorithm http://paulbourke.net/geometry/pointlineplane/
  //line segments are broken to the line segments of form P = P1 + alpha(P2-P1) and P = P3 + beta(P4-P3)
  // alpha and beta will be between 0-1 if the interception is inside the line segment 
  double alpha = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) /
                 ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));
  double beta = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) /
                ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));

  //if both alpha and beta are between 0-1 then they intercept within the cell
  return (((alpha > 0) && (alpha < 1)) && ((beta > 0) && (beta < 1)));
}

/**
 * @brief check if point intercepts a triangle
 * 
 * @param triangle_p1 point of triangle
 * @param triangle_p2 point of triangle
 * @param triangle_p3 point of triangle
 * @param test_p point to be tested
 * @return true if test point intercepts triangle
 * @return false if test point doesnt intercept triangle
 */
bool cone_intersection_(const Point& triangle_p1, const Point& triangle_p2, const Point& triangle_p3, const Point& test_p)
{
  //triangle point interception as per method 3 found here https://totologic.blogspot.com/2014/01/accurate-point-in-triangle-test.html
  // v1,v2,v3 are all vectors but to avoid unnecessary implemtaion typed as points
  Point v1 = {(triangle_p2.y - triangle_p1.y), (-triangle_p2.x, triangle_p1.x)};
  Point v2 = {(triangle_p3.y-triangle_p2.y), (-triangle_p3.x + triangle_p2.x)};
  Point v3 = {(triangle_p1.y-triangle_p3.y), (-triangle_p1.x+ triangle_p3.x)};

  // v1_prime,v2_prime,v3_prime are all vectors but to avoid unnecessary implemtaion typed as points
  Point v1_prime = {(test_p.x-triangle_p1.x), (test_p.y - triangle_p1.y)};
  Point v2_prime = {(test_p.x-triangle_p2.x), (test_p.y - triangle_p2.y)};
  Point v3_prime = {(test_p.x-triangle_p3.x), (test_p.y - triangle_p3.y)};

  // obtaining the dot produt of each vector with its assosiated prime vector
  double dot1 = (v1.x*v1_prime.x)+(v1.y*v1_prime.y); 
  double dot2 = (v2.x*v2_prime.x)+(v2.y*v2_prime.y); 
  double dot3 = (v3.x*v3_prime.x)+(v3.y*v3_prime.y);

  //if any dot product is greater than or equal to zezo then it intercepts or is inside the triangle
  return ((dot1 >= 0) && (dot2 >= 0) && (dot3 >= 0));
}

