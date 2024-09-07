#ifndef CIRCLE_H
#define CIRCLE_H
//Implement a circle class that has:
//* A constructor that sets the radius
//* A Method that sets the radius
//* A Method that returns the area
//* A Method that returns the perimeter



class Circle
{
public:
  Circle(double radius);
  void setRadius(double radius);
  double area(void);
  double perimeter(void);

private:
  double radius_;

};

#endif // CIRCLE_H
