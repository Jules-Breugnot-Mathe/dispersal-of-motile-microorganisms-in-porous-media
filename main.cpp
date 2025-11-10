#include <iostream>
#include <vector>
#include <memory>
#include <random>


#include "Point.hpp"
#include "Solid.hpp"
#include "Disk.hpp"
#include "Mobile.hpp"
#include "Environment.hpp"
#include "Rectangle.hpp"
#include "Domain.hpp"
#include "tests.hpp"


using namespace std;




int main(){
      
    
    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(std::random_device{}());
    
    Point P(0, 0);
    Environment E_empty("empty");

    Environment E1("uniform_Disks", 1, 0.4);
    Environment E2("uniform_Disks", 1, 0.2);
    Environment E3("uniform_Squares", 1, 0.3);
    Environment E4("uniform_Disks", 1, 0.2);

    Mobile M(P, 1, 0, 1, 0, 0.5); // v0=10, Dr=0, Tau=1, Theta=0, mu = 0.7

    //M.diffusivity_function_of_tau(E1, P, "isotropic", "Diffusivity_RR_large_uniform_Disks.csv", 10, 1000, 20);

    double D = M.measure_diffusivity_expo(E_empty, P,"isotropic", 1000);
    cout<<D<<endl;



    
    return 0; 
}

