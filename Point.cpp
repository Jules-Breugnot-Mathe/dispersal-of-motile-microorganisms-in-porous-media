#include "Point.hpp"
#include <iostream>
#include <ostream>

Point::Point(double x, double y){
    this->x = x; this->y = y;
}


Point::~Point(){}

Point::Point(const Point & other){
    this->x = other.x;
    this->y = other.y;
}

Point Point::operator=(const Point & other){
    this->x = other.x;
    this->y = other.y;
    return *this;
}

const double Point::getx() const{
    return this->x;
}

const double Point::gety() const{
    return this->y;
}

Point Point::operator+(const Point & other) const{
    return Point(this->x + other.getx(), this->y + other.gety());
}

Point Point::operator-(const Point & other){
    this->x -= other.getx();
    this->y -= other.gety();
    return *this;
}

std::ostream& operator<<(std::ostream& o, const Point & pt){
    o << "( " <<pt.x<< " , "<<pt.y<<" )"<<std::endl;
    return o;
}

Point Point::operator*(const double a) const {
    return Point(x * a, y * a);
}

const double Point::dot_product(const Point & other) const {
    return x*other.getx() + y*other.gety();
}

