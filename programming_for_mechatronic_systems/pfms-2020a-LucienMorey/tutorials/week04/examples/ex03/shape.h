#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <cmath>

/*!
 *  \ingroup   ac_shape Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    Shape();
    //virtual ~Shape();
    void setCentre(double x, double y);
    std::string getDescription();
    void offset(double x, double y);
    virtual double getArea() = 0;
    //! Adding a virtual for checking intersect
    virtual bool checkIntercept(double x, double y) =0;
protected:
    std::string description_;//!< description of shape
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
