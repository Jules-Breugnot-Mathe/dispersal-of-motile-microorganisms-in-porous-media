#include "Domain.hpp"
#include "Mobile.hpp"

Domain::Domain(double r){
    this->radius = r;
}

Domain::~Domain(){
}

Domain::Domain(const Domain & other){
    this->radius = other.radius;
}

Domain& Domain::operator=(const Domain & other){
    this->radius = other.radius;
    return *this;
}

const double & Domain::getradius(){
    return this->radius;
}

void Domain::MakeLoop(Mobile & M) const {
    //Loop stocke le nombre de tour sur le Tore, en première position sur l'axe
    //(Ox) et en deuxième position sur l'axe (OY). On incrémente positivement (resp. négativement)
    //lorsque le mobile dépasse positivement le rayon du domaine (resp. négativement)

    //cette méthode assure que Coord reste dans le domaine fondamental, que on a bien compté le dépassement du bord, 
    //et que Free_coord représente bien les coordonnées libres dans R2

    if (M.getCoord().getx() > radius){
        M.Loop.data()[0]++;
        M.Coord = M.Coord + Point(-2*radius, 0);
        //std::cout<<" a droite"<<std::endl;
    }
    if (M.getCoord().getx() < -radius){
        M.Loop.data()[0]--;
        M.Coord = M.Coord + Point(2*radius, 0);
        //std::cout<<" a gauche"<<std::endl;
    }
    if (M.getCoord().gety() > radius){
        M.Loop.data()[1]++;
        M.Coord = M.Coord + Point(0, -2*radius);
        //std::cout<<" en haut"<<std::endl;
    }
    if (M.getCoord().gety() < -radius){
        M.Loop.data()[1]--;
        M.Coord = M.Coord + Point(0, 2*radius);
        //std::cout<<" en bas"<<std::endl;
    }
    M.Free_coord = M.Coord + Point(M.Loop.data()[0], M.Loop.data()[1])*(2*radius);
}
