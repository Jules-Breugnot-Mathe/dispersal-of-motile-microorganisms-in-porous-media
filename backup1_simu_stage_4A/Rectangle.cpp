#include "Rectangle.hpp"


Rectangle::Rectangle(Point p1, Point p2, Point p3, Point p4){
    tab[0] = p1, tab[1] = p2; tab[2] = p3; tab[3] = p4;
}

Rectangle::~Rectangle(){}

Rectangle::Rectangle(const Rectangle & other){
    for (int j=0;j<4;j++){
        this->tab[j] = other.tab[j];
    }
}

Rectangle Rectangle::operator=(const Rectangle & other){
    for (int j=0;j<4;j++){
        this->tab[j] = other.tab[j];
    }
    return *this;
}


bool Rectangle::IsCollided(const Mobile & M){
    return false; // à modif plus tard
}


double* Rectangle::escapeAngle(const Mobile & M){
    double* I = new double[2];
    //reste à calculer précisemment les valeurs inf et sup
    return I;
}


void Rectangle::edgeProjection(Mobile & M){
    //reste à calculer 
}


