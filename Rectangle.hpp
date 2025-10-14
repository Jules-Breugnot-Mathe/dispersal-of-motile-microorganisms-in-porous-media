#ifndef RECTANGLE_HPP
#define RECTANGLE_HPP

#include "Solid.hpp"
#include "Point.hpp"
#include "Mobile.hpp"

//tests unitaires effectués

class Rectangle : public Solid{
    private : 
        Point tab[4];
    public : 
        Rectangle(Point p1 = Point(), Point p2 = Point(), Point p3 = Point(), Point p4 = Point());
        ~Rectangle();
        Rectangle(const Rectangle & other);
        Rectangle operator=(const Rectangle & other);
        bool IsCollided(const Mobile & M) override;
        double* escapeAngle(const Mobile & M) override;
        void edgeProjection(Mobile & M) override;
};


#endif //RECTANGLE_HPP