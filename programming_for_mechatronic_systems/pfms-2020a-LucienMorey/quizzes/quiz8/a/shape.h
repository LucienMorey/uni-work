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
    Shape();
    void setCentre(double x, double y);
    void getCentre(double& x, double& y);
    std::string getDescription();
    void offset(double x, double y);
    //!TODO - TASK 1:  Modify the base class `Shape` such that function getArea() is defined in `Shape`, and the child classes are required to implement this function.
    virtual double getArea()=0;
    virtual bool checkIntercept (double x, double y)=0;

protected:
    std::string description_;//!< description of shape
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
