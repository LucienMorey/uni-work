#include "analysis.h"

#include <iostream>
#include <math.h>
#include "circle.h"

Analysis::Analysis()
{
}

//! TODO - TASK 3: Implement the missing function(s) in Analysis Class
//!
//! HINT: Analysis inherits from AnalysisInterface which is an abstract class
//! What type of class is AnalysisInterface, what does this mean?
//!
//! Use the following webiste to assist in developing the code
//! https://mathworld.wolfram.com/Circle-LineIntersection.html
std::vector<bool> Analysis::intersectsLine()
{
  std::vector<bool> intersection;

  double delta_x = 1.0 - 0.0;
  double delta_y = line_.calculateYValue(1.0) - line_.calculateYValue(0.0);
  double delta_r = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
  double determinent = 1.0 * line_.calculateYValue(0.0) - 0.0 * line_.calculateYValue(1.0);
  for (auto shape : shapes_)
  {
    if (shape->getDescription() == "circle")
    {
      double discriminent = pow(sqrt(dynamic_cast<Circle*>(shape)->getArea()/M_PI), 2) * pow(delta_r, 2) - pow(determinent, 2);
      intersection.push_back(discriminent >= 0);
    }
    else
    {
      intersection.push_back(false);
    }
  }
  return intersection;
}
//!
//! BONUS QUESTION (FOR DISCUSSION)
//! At the moment we are implementing check intersect for a Circle
//! as the only question used.
//! If we had a Rectangle shape, how could we differentiate between the approach
//! to intersect for the Rectangle and the Circle?
