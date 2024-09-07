#include "rangerfusion.h"

/**
 * @brief rangers setter. sets class pointer to vector of rangers for use in fusion class
 *
 * @param rangers vector of ranger pointers to be set
 */
void RangerFusion::setRangers(std::vector<RangerInterface*> rangers)
{
  rangers_ = rangers;
}

/**
 * @brief cells setter. sets class pointer to cells for fusion
 *
 * @param cells vector of cell pointers to use
 */
void RangerFusion::setCells(std::vector<Cell*> cells)
{
  cells_ = cells;
}

/**
 * @brief calls sensors to generate data to be stored in data. Checks each cell for occupancy based on data generated.
 *
 */
void RangerFusion::grabAndFuseData()
{
  // clear previous raw data
  data_.clear();
  // generate data from all rangers
  for (auto ranger : rangers_)
  {
    data_.push_back(ranger->generateData());
  }

  // reset state of cells now that new data has been
  for (auto cell : cells_)
  {
    cell->setState(UNKNOWN);
  }
  // iterate through data and rangers to determine state of each cell
  auto data_it = data_.begin();
  auto ranger_it = rangers_.begin();
  for (; ((data_it < data_.end()) && (ranger_it < rangers_.end())); ++ranger_it, ++data_it)
  {
    // check which sensor the rangers points to and determine if laser or sonar tests are appropriate
    if ((*ranger_it)->getSensingMethod() == POINT)
    {
      laserCheck_((*data_it), (*ranger_it));
    }
    else
    {
      sonarCheck_((*data_it), (*ranger_it));
    }
  }
}

/**
 * @brief raw data getter. will return the last set of raw data based on grabAndFuseData
 *
 * @return std::vector<std::vector<double>>  vector contaiing each vector of sensor data
 */
std::vector<std::vector<double>> RangerFusion::getRawRangeData()
{
  return data_;
}

/**
 * @brief Check for cell state based on laser data state. occupies if laser point is within cell bounds. free if laser
 * line intercepts a line created by cell diagonals
 *
 * @param data vector of doubles containing the laser data. length varies based on angle resolution
 * @param ranger ranger sensor containing data. will only work correctly if sensor is of laser subclass
 */
void RangerFusion::laserCheck_(std::vector<double> data, RangerInterface* ranger)
{
  for (auto cell : cells_)
  {
    // reset laser angle for new cell
    int laser_angle_ = 0 + ranger->getOffset();

    // calculate midpoint of cell
    double mid_x, mid_y;
    cell->getCentre(mid_x, mid_y);

    // calculate the corner points of the cell for testing
    Point cell_top_left = { (mid_x - cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_top_right = { (mid_x + cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_bottom_left = { (mid_x - cell->getSide() / 2), (mid_y - cell->getSide() / 2) };
    Point cell_bottom_right = { (mid_x + cell->getSide() / 2), (mid_y - cell->getSide() / 2) };

    // check each datapoint for ech cell
    for (auto data_point : data)
    {
      // if previously marked as occupied, increment angle resolution break immideately
      if (cell->getState() == OCCUPIED)
      {
        // increment angle of reading by resolution
        laser_angle_ += ranger->getAngularResolution();
        break;
      }

      // obtain co-ordinates of data point
      Point laser_point = { data_point * cos(degToRads_(laser_angle_)), data_point * sin(degToRads_(laser_angle_)) };

      // increment the angle of reading by resolution
      laser_angle_ += ranger->getAngularResolution();

      // the point is inside the cell then it is occupied and the loop can be broken
      if (pointCellIntersection_(laser_point, cell))
      {
        cell->setState(OCCUPIED);
        break;
      }
      // if line segments intersect and  fails the point interception test then its a free
      // loop cannot be broken if cell is marked as free in case cell is marked occupied by other data
      else if ((lineLineIntersection_(laser_point, ORIGIN, cell_top_left, cell_bottom_right)) ||
               (lineLineIntersection_(laser_point, ORIGIN, cell_bottom_left, cell_top_right)))
      {
        cell->setState(FREE);
      }
      // if both tests fail then the cell remains an unknown state
    }
  }
}

/**
 * @brief check cell state based on sonar data. free if cell equal isos lines intercept cell or if cell lies inside
 * triangle. occupied if third line intercepts or the two joining points lie in the cell
 *
 * @param data vector of doubles containing data from a sonar sensor
 * @param ranger ranger interface pointer pointing to sonar subclass
 */
void RangerFusion::sonarCheck_(std::vector<double> data, RangerInterface* ranger)
{
  for (auto cell : cells_)
  {
    // calculate angle that sensor is sitting at
    // sonar sits at 90 degrees by default
    double sonar_angle = 90 + ranger->getOffset();

    // calculate midpoint of cell
    double mid_x, mid_y;
    cell->getCentre(mid_x, mid_y);

    // calculate cell points for checking if cell is occupied
    Point cell_mid = { mid_x, mid_y };
    Point cell_top_left = { (mid_x - cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_top_right = { (mid_x + cell->getSide() / 2), (mid_y + cell->getSide() / 2) };
    Point cell_bottom_left = { (mid_x - cell->getSide() / 2), (mid_y - cell->getSide() / 2) };
    Point cell_bottom_right = { (mid_x + cell->getSide() / 2), (mid_y - cell->getSide() / 2) };
    for (auto data_point : data)
    {
      // break immediately if cell is already occupied
      if (cell->getState() == OCCUPIED)
      {
        break;
      }

      // data reading is perpendicular distance between origin and centre of triangle side.
      double data_hyp = data_point / cos(degToRads_(ranger->getFieldOfView() / 2));

      // location of the two sonar vertexes can be found after obtaining the length of the eequal triangle sides
      // One point will be sitting at the angle of the sonar + angleRes/2 and the other will be at the the angle of the
      // sonar - angleRes/2
      Point sonar_point_1 = { data_hyp * cos(degToRads_(sonar_angle + (ranger->getAngularResolution() / 2))),
                              data_hyp * sin(degToRads_(sonar_angle + (ranger->getAngularResolution() / 2))) };

      Point sonar_point_2 = { data_hyp * cos(degToRads_(sonar_angle - (ranger->getAngularResolution() / 2))),
                              data_hyp * sin(degToRads_(sonar_angle - (ranger->getAngularResolution() / 2))) };

      // if either of the two points sits in the cell or the line joining them crosses the cell diagonals then the cell
      // is occupied
      // if the cell is occupied the loop can break immediately
      if (lineLineIntersection_(sonar_point_1, sonar_point_2, cell_bottom_right, cell_top_left) ||
          lineLineIntersection_(sonar_point_1, sonar_point_2, cell_top_right, cell_bottom_left) ||
          pointCellIntersection_(sonar_point_1, cell) || pointCellIntersection_(sonar_point_2, cell))
      {
        cell->setState(OCCUPIED);
        break;
      }
      // if either of the two equal side eges intersect the diagonals or the point lies inside the triangle  and the
      // occupied test fails its a free cell
      else if (pointTriangleIntersection_(sonar_point_1, sonar_point_2, cell_mid) ||
               lineLineIntersection_(ORIGIN, sonar_point_1, cell_top_left, cell_bottom_right) ||
               lineLineIntersection_(ORIGIN, sonar_point_1, cell_bottom_left, cell_top_right) ||
               lineLineIntersection_(ORIGIN, sonar_point_2, cell_top_left, cell_bottom_right) ||
               lineLineIntersection_(ORIGIN, sonar_point_2, cell_bottom_left, cell_top_right))

      {
        cell->setState(FREE);
      }
    }
    // if the cell is makred as free or remains unknown then the nextdata point needs to be tested in case of occupancy
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
bool RangerFusion::pointCellIntersection_(const Point& point, Cell* cell)
{
  // obtain midpoint to easily find vertexes of cell
  double mid_x, mid_y;
  cell->getCentre(mid_x, mid_y);

  // boundaries are found with midpoint +- the cell side length
  // returns true if point intersects with cell
  return (point.x <= (mid_x + (cell->getSide() / 2))) && (point.x >= (mid_x - (cell->getSide() / 2))) &&
         (point.y <= (mid_y + (cell->getSide() / 2))) && (point.y >= (mid_y - (cell->getSide() / 2)));
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
bool RangerFusion::lineLineIntersection_(const Point& p1, const Point& p2, const Point& p3, const Point& p4)
{
  // credit to Paulborke for algorithm http://paulbourke.net/geometry/pointlineplane/
  // line segments are broken to the line segments of form P = P1 + alpha(P2-P1) and P = P3 + beta(P4-P3)
  // alpha and beta will be between 0-1 if the interception is inside the line segment
  double denominator = ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));
  double alpha_numerator = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x));
  double beta_numerator = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x));
  // check for division by zero
  if (denominator == 0.0)
  {
    // if both numerators and the denominator are zero then the lines are coincident
    // else the lines are parallel and return false to check next
    if ((alpha_numerator == 0.0) && (beta_numerator == 0.0))
    {
      return true;
    }
    return false;
  }

  double alpha = alpha_numerator / denominator;
  double beta = beta_numerator / denominator;

  // if both alpha and beta are between 0-1 then they intercept within the cell
  return ((alpha > LINE_SEGMENT_RATIO_BEGINNING) && (alpha < LINE_SEGMENT_RATIO_ENDING)) &&
         ((beta > LINE_SEGMENT_RATIO_BEGINNING) && (beta < LINE_SEGMENT_RATIO_ENDING));
}

/**
 * @brief check if point intercepts a triangle
 *
 * @param triangle_p1 point of triangle
 * @param triangle_p2 point of triangle
 * @param test_p point to be tested
 * @return true if test point intercepts triangle
 * @return false if test point doesnt intercept triangle
 */
bool RangerFusion::pointTriangleIntersection_(const Point& triangle_p1, const Point& triangle_p2, const Point& test_p)
{
  // check if distance between origin and point is less than distance between origin and either of the other two
  // triangle points if yes then the point has potential to be inside the triangle
  if (sqrt(pow(test_p.x, 2) + pow(test_p.y, 2)) < (sqrt(pow(triangle_p2.x, 2) + pow(triangle_p2.y, 2))))
  {
    // check location of test point and triangle points
    // if they lie in diagonally opposite quadrants then check was a false positive
    if (((checkQuadrant_(test_p) == FIRST) && (checkQuadrant_(triangle_p1) != THIRD) &&
         (checkQuadrant_(triangle_p2) != THIRD)) ||
        ((checkQuadrant_(test_p) == SECOND) && (checkQuadrant_(triangle_p1) != FOURTH) &&
         (checkQuadrant_(triangle_p2) != FOURTH)) ||
        ((checkQuadrant_(test_p) == THIRD) && (checkQuadrant_(triangle_p1) != FIRST) &&
         (checkQuadrant_(triangle_p2) != FIRST)) ||
        ((checkQuadrant_(test_p) == FOURTH) && (checkQuadrant_(triangle_p1) != SECOND) &&
         (checkQuadrant_(triangle_p2) != SECOND)))
    {
      // calculate all line gradients. all lines are assumed to have started from the origin
      double triangle_line_1_gradient = atan2(triangle_p1.y, triangle_p1.x);
      double triangle_line_2_gradient = atan2(triangle_p2.y, triangle_p2.x);
      double test_line_gradient = atan2(test_p.y, test_p.x);

      // if the point gradient lies between the other two the point intercepts.
      // if one of the triangle lines lies in the second and the other in the third quadrant then one gradient will be
      // negative and the other positive in this case the point lies in the triangle if the test gradient is greater
      // than both triangle gradients
      if ((triangle_line_2_gradient < test_line_gradient) && (triangle_line_1_gradient > test_line_gradient))
      {
        return true;
      }
      else if ((checkQuadrant_(triangle_p1) == THIRD) && (checkQuadrant_(triangle_p2) == SECOND))
      {
        if (((triangle_line_2_gradient < test_line_gradient) && (triangle_line_1_gradient < test_line_gradient)) ||
            ((triangle_line_2_gradient > test_line_gradient) && (triangle_line_1_gradient > test_line_gradient)))
        {
          return true;
        }
      }
    }
  }

  return false;
}

/**
 * @brief function takes a Point and will calculate the angle it makes with positive x-axis and return the quadrant it
 * lies in
 *
 * @param point point in cartesian space to be tested
 * @return quadrant enum of quadrants based on angle generated between point and origin
 */
quadrant RangerFusion::checkQuadrant_(const Point& point)
{
  // use a atan2 to get result between pi and -pi and determine quadrant
  double angle = atan2(point.y, point.x);
  if ((angle >= 0) && (angle <= M_PI / 2))
  {
    return FIRST;
  }
  else if ((angle >= M_PI / 2) && (angle <= M_PI))
  {
    return SECOND;
  }
  else if ((angle <= 0) && (angle >= -M_PI / 2))
  {
    return THIRD;
  }
  else if ((angle <= -M_PI / 2) && (angle >= M_PI))
  {
    return FOURTH;
  }
}

/**
 * @brief converts degrees to radians
 *
 * @param degs int number of degrees to be converted
 * @return double converted angle in radians
 */
double RangerFusion::degToRads_(int degs)
{
  return (M_PI / 180) * degs;
}
