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

    Environment E1("uniform_Disks", 1, 0.4); // large uniform Disks
    Environment E2("uniform_Disks", 1, 0.2); // small uniform Disks
    Environment E3("uniform_Squares", 1, 0.4); // large uniform Squares
    Environment E4("uniform_Squares", 1, 0.2); // small uniform Squares

    Mobile M(P, 1, 0, 1, 0, 0.5); // v0=10, Dr=0, Tau=1, Theta=0, mu = 0.7


    //M.write_trajectory_expo(E3, 100, 0.01, P, seed, "isotropic", "C:\\Users\\Jules\\Desktop\\Stage ENS 4A\\data\\Trajectories.csv");
    /*
    // large uniform Disks
    M.diffusivity_function_of_tau(E1, P, "isotropic", "Diffusivity_IS_large_uniform_Disks.csv", 100, 1000, 20);
    M.diffusivity_function_of_tau(E1, P, "run_and_reverse", "Diffusivity_RR_large_uniform_Disks.csv", 100, 1000, 20);
    // small uniform Disks
    M.diffusivity_function_of_tau(E2, P, "isotropic", "Diffusivity_IS_small_uniform_Disks.csv", 100, 1000, 20);
    M.diffusivity_function_of_tau(E2, P, "run_and_reverse", "Diffusivity_RR_small_uniform_Disks.csv", 100, 1000, 20);
    */
    
    // large uniform Squares
    //M.diffusivity_function_of_tau(E3, P, "isotropic", "Diffusivity_IS_large_uniform_Squares.csv", 10, 1000, 10, 0.82690);
    //M.diffusivity_function_of_tau(E3, P, "run_and_reverse", "Diffusivity_RR_large_uniform_Squares.csv", 100, 1000, 20);
    // small uniform Squares
    M.diffusivity_function_of_tau(E4, P, "isotropic", "Diffusivity_IS_small_uniform_Squares.csv", 200, 1000, 20, 0.82690);
    // M.diffusivity_function_of_tau(E4, P, "run_and_reverse", "Diffusivity_RR_small_uniform_Squares.csv", 100, 1000, 20);
    
    return 0; 
}

