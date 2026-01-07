#ifndef RECTANGLE_HPP
#define RECTANGLE_HPP

#include "Solid.hpp"
#include "Point.hpp"
#include "Mobile.hpp"

#include <ostream>

//tests unitaires effectués

class Rectangle : public Solid{
    private : 
        Point vertex[4]; 
        Point axis[4];

    public : 
        Rectangle(Point p1 = Point(), Point p2 = Point(), Point p3 = Point(), Point p4 = Point());
        Rectangle(Point center = Point(), double radius = 0);
        ~Rectangle();
        Rectangle(const Rectangle & other);
        friend std::ostream& operator<<(std::ostream& o, const Rectangle & rec);
        Rectangle operator=(const Rectangle & other);
        bool IsCollided(const Mobile & M) override;
        double* escapeAngle(const Mobile & M) override;
        void edgeProjection(Mobile & M) override;

        const Point& getVertex(int i) const;
};


#endif //RECTANGLE_HPP