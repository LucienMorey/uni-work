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
    std::string getDescription();
    void offset(double x, double y);
    //TASK 1:  Modify the base class `Shape` such that function getArea() is defined in `Shape`, and the child classes are required to implement this function.
    // By declaring it as virtual, and using the `= 0` syntax, getArea is now a
    // pure virtual function. 
    virtual double getArea() = 0;

protected:
    std::string description_;//!< description of shape
private:
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
