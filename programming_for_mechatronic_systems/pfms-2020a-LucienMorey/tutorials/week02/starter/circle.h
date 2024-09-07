#ifndef CIRCLE_H
#define CIRCLE_H

class Circle{
    public:
        Circle(double radius);
        void set_radius(double radius);
        double get_area();
        double get_perimeter();

    private:
        double radius;
};

#endif