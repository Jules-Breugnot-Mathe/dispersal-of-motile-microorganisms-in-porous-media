#include "Environment.hpp"
#include "Solid.hpp"
#include "Disk.hpp"
#include <vector>


Environment::Environment(std::string Type, int radius, double R){
    if (Type == "empty") {
    }
    else if (Type == "uniform_Disks") {
        int i;
        int j;
        for (int i=0; i<radius; i++){
            for (int j=0; j<radius; j++){
                Solid_vector.push_back(std::make_unique<Disk>(Point(0.5 + i, 0.5 + j), R)); 
            }
        }
        for (int i=0; i<radius; i++){
            for (int j=0; j<radius; j++){
                Solid_vector.push_back(std::make_unique<Disk>(Point(-0.5 - i, 0.5 + j), R)); 
            }
        }
        for (int i=0; i<radius; i++){
            for (int j=0; j<radius; j++){
                Solid_vector.push_back(std::make_unique<Disk>(Point(0.5 + i, -0.5 - j), R)); 
            }
        }
        for (int i=0; i<radius; i++){
            for (int j=0; j<radius; j++){
                Solid_vector.push_back(std::make_unique<Disk>(Point(-0.5 - i, -0.5 - j), R));
            }
        }
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


