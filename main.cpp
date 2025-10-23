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
    Environment E("empty", 1, 0.4);
    Mobile M(P, 1, 0, 1, 0, 0.5); // v0=10, Dr=0, Tau=1, Theta=0, mu = 0.7

    
    //M.write_trajectory(E, 100, 0.01 , P, seed, "isotropic", "Trajectories.csv");
    //cout<<"deplacement final : "<<M.getMt()<<endl;
    /*
    double D = 0;
    int N = 50;
    for (int i=0 ; i<N ; i++){
        M.simulation(E, 50000, 0.01, Point(0,  0), seed+i, "isotropic");
        cout<<M.getMt()<<endl;
        D += (M.getMt()*M.getMt())/(4*50000);
    }
    
    cout<<D/N<<endl;
    */
    //M.simulation(E, 34792, 0.01, Point(0,  0), seed, "isotropic");
    //std::cout<<(M.getMt()*M.getMt())/(4*34792)<<std::endl;

    //M.test_exponentiality(E, 200, 0.001 , P, seed, "isotropic", "Tau_estimation_data.csv", 1);
    M.measure_diffusivity(E, 0.1, P, "isotropic", "D_estimation_data.csv", 100000, 2000, 1000);
    //M.measure_displacement(E, 34792, 0.01, P, "isotropic", "measure_displacement.csv", 1000);


    return 0; 
}

