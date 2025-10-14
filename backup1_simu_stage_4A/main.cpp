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
#include "tests.hpp"


using namespace std;




int main(){
    
    /*
    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(std::random_device{}());
    
    Point P(0, 0);
    Environment E("uniform_Disks", 10, 0.4);
    Mobile M(P, 1, 0, 1, 0, 0.5); // v0=10, Dr=0, Tau=1, Theta=0, mu = 0.7
    M.write_trajectory(E, 500, 0.001 , P, seed, "run_and_reverse", "Trajectories.csv");
    cout<<"Coordonnees finales : "<<M.getCoord()<<endl;  
    cout<<"Deplacement Total : "<<M.getMt()<<endl; 
    */
    
    

    /*
    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(std::random_device{}());
    Environment E("empty");
    double R = std::sqrt(2)/2;
    Disk D1(Point(1, 1), R), D2(Point(-1, 1), R), D3(Point(1, -1), R), D4(Point(-1, -1), R);
    D1.getid() = 1;  D2.getid() = 2;  D3.getid() = 3;  D4.getid() = 4;
    E.get_Solid_vector().push_back(std::make_unique<Disk>(D1));
    E.get_Solid_vector().push_back(std::make_unique<Disk>(D2));
    E.get_Solid_vector().push_back(std::make_unique<Disk>(D3));
    E.get_Solid_vector().push_back(std::make_unique<Disk>(D4));
    Mobile M(Point(0, 0), 1, 0.0, 0.1, 0, 0.7);
    M.simulation(E, 100, 0.1, Point(0,  0), seed, "isotropic");
    cout<<"Coordonnees finales : "<<M.getCoord()<<endl;  
    cout<<"Deplacement Total : "<<M.getMt()<<endl; 
    */
    
    /*
    Mobile M(Point(0, 0), 1, 0, 1, 0, 0.5);
    Environment E;
    Disk D(Point(1, 0), 0.5);

    E.get_Solid_vector().push_back(std::make_unique<Disk>(D));
    M.simulationV2(E, 1, 0.01, Point(0, 0), "Trajectories.csv");
    */

    
    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(std::random_device{}());
    
    Point P(0, 0);
    Environment E("uniform_Disks", 10, 0.4);
    Mobile M(P, 1, 0, 1, 0, 0.5); // v0=10, Dr=0, Tau=1, Theta=0, mu = 0.7
    M.test_exponentiality(E, 200, 0.001 , P, seed, "isotropic", "Tau_estimation_data.csv", 1);
    
    
    return 0; 
}

