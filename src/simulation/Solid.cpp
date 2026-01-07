#include "Solid.hpp"
#include "Mobile.hpp"


Solid::Solid(){
}

Solid::~Solid(){
}

const int Solid::getid() const{
    return this->id;
}

void Solid::Coordoverwrite(Mobile & M, Point P){
    M.Coord = P;
}

int & Solid::getid(){
    return this->id;
}


