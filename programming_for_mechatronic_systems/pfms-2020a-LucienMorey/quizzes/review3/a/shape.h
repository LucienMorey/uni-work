#ifndef SHAPE_H
#define SHAPE_H

#include <string>

/*!
 *  \ingroup   ac_shape Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    Shape(double centreX, double centreY);
    ~Shape();
    virtual void setCentre(double x, double y);
    std::string getDescription();
    virtual void offset(double x, double y);
    virtual double getArea() = 0;
    virtual bool checkPoint(double x, double y) = 0;
protected:
    std::string description_;//!< description of shape
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
