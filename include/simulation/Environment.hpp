#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <vector>
#include <string>
#include "Solid.hpp"
#include <memory>
#include "Point.hpp"
#include "Domain.hpp"

class Environment{
    private : 
        std::vector<std::unique_ptr<Solid>> Solid_vector;
        Domain D;
    public : 
        Environment(std::string Type = "empty", int radius = 1, double R = 0.3);
        ~Environment();
        const std::vector<std::unique_ptr<Solid>> & get_Solid_vector() const ;
        std::vector<std::unique_ptr<Solid>> & get_Solid_vector();
        const Domain & getDomain() const ;
};



#endif //ENVIRONMENT_HPP