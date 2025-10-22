#include "Environment.hpp"
#include "Solid.hpp"
#include "Disk.hpp"
#include <vector>


Environment::Environment(std::string Type, int radius, double R){
    this->D = Domain(radius); // radius représente le rayon du domaine fondamental, R le rayon des obstacles
    if (Type == "empty") {
    }
    else if (Type == "uniform_Disks") {
        Solid_vector.push_back(std::make_unique<Disk>(Point(0.5*radius, 0.5*radius), R));
        Solid_vector.push_back(std::make_unique<Disk>(Point(-0.5*radius, -0.5*radius), R));
        Solid_vector.push_back(std::make_unique<Disk>(Point(-0.5*radius, 0.5*radius), R));
        Solid_vector.push_back(std::make_unique<Disk>(Point(0.5*radius, -0.5*radius), R));
    }
}

Environment::~Environment(){}



const std::vector<std::unique_ptr<Solid>> & Environment::get_Solid_vector() const{ 
    // accesseur en lecture seule
    return this->Solid_vector;
}


std::vector<std::unique_ptr<Solid>> & Environment::get_Solid_vector(){ 
    // accesseur en lectture et écriture, pour rajouter des Solid par exemple
    return this->Solid_vector;
}

const Domain & Environment::getDomain() const {
    return this->D;
}

