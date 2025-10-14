#ifndef POINT_HPP
#define POINT_HPP
#include <iostream>
#include <ostream>

//tests unitaires effectués

class Point{
    private : 
        double x;
        double y;
    public : 
        Point(double x = 0, double y = 0);
        ~Point();
        Point(const Point & other);
        Point operator=(const Point & other);
        const double getx() const;
        const double gety() const;
        Point operator+(const Point & other);
        friend std::ostream& operator<<(std::ostream& o, const Point & pt);
        Point operator-(const Point & other);
        Point operator*(const double a) const;
};

#endif //POINT_HPP

