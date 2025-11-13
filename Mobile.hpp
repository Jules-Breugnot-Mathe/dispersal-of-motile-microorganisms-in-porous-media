#ifndef MOBILE_HPP
#define MOBILE_HPP
#include "Point.hpp"
#include "Solid.hpp"
#include "Disk.hpp"
#include "Domain.hpp"

#include <memory>
#include <random>
#include <string>
#include <list>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>
#include <iomanip>
#include <array>
#include <algorithm>


class Environment;


//tests unitaires effectués

class Mobile{
    private : 
        double v0; // vitesse à l'origine
        double Dr; // diffusion rotationelle
        double Tau; // moyenne d'attente de l'évènement tumbling

        Point Coord; // coordonnées sur le tore, modulo le rayon
        Point Free_coord; // coordonnées libres dans R2
        std::array<double, 2> Loop; // sert à compter le nombre de tour sur le tore

        double Theta; // angle avec l'axe Ux
        int PoissonCount; // valeur du processus de comptage d'évènement "tumbling"
        double mu; //escape rate dans le cas d'une simulation en situation de non glissement (c'est en fait ici 1/mu)
        double time; // temps de simulation
        double Mt; // observation du déplacement total du processus au temps t
        bool IsCollided; // détection des collisions, processus aléatoire 
        friend void Solid::Coordoverwrite(Mobile & M, Point P);
        friend void Domain::MakeLoop(Mobile & M) const ;

    public : 
        Mobile(Point P = Point(0, 0), double v0 = 1, double Dr = 0, double Tau = 1, double Theta = 0, double mu = 0.5);
        Mobile(const Mobile & other);
        ~Mobile();
        Mobile operator=(const Mobile & other);
        Point getCoord() const; // autorise à utiliser Coord dans un calcul, pas à modifier la valeur d'un objet Mobile
        Point getFreeCoord() const; // pareil
        void simulation(const Environment & E, double T=1, double h=0.01, Point X = Point(), uint64_t seed = 0, std::string Reorientation_mode = "isotropic");
        Solid* CollisionDetection(const Environment & E);
        const double getMt() const;
        void simulation_expo(const Environment & E, double T=1, double h=0.01, Point X = Point(), uint64_t seed = 0, std::string Reorientation_mode = "isotropic");
        void write_trajectory_expo(const Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename);
        void write_trajectory(const Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename);
        Point& getCoord();
        Point& getFreeCoord();
        double getV0ForDebug() const; 
        double getThetaForDebug() const;
        void test_exponentiality(Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename, int N_samples);
        void measure_diffusivity(Environment & E, Point X, std::string Reorientation_mode, const std::string & filename, int N_samples, int N_data);
        double measure_diffusivity_expo(const Environment & E, Point X, std::string Reorientation_mode, int N_samples);
        void diffusivity_function_of_tau(const Environment & E, Point X, std::string Reorientation_mode, const std::string & filename, double tau_upper_bound, int N_samples, int N_data, double tau_star);
        void measure_displacement(Environment & E, double T, double h, Point X, std::string Reorientation_mode, const std::string & filename, int N_samples);
        std::array<double, 2> max_tau_bissection_approx(const Environment & E, Point X, std::string Reorientation_mode, double tau_upper_bound, int N_samples, double tol);
    };


#endif //MOBILE_HPP