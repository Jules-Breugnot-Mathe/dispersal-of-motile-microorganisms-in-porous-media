#ifndef DISK_HPP
#define DISK_HPP
#include "Solid.hpp"
#include "Point.hpp"
#include "Mobile.hpp"

//tests unitaires effectués

class Disk: public Solid{
    private : 
        Point center;
        double radius;
    public : 
        Disk(Point center = Point(), double radius = 0);
        ~Disk();
        Disk(const Disk & other);
        Disk operator=(const Disk & other);
        bool IsCollided(const Mobile & M) override;
        double* escapeAngle(const Mobile & M) override;
        void edgeProjection(Mobile & M) override;

        const Point& getCenter() const;
        double getRadius() const;
        
};


#endif //DISK_HPP