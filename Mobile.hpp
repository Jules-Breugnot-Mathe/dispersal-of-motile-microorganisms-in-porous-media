#ifndef MOBILE_HPP
#define MOBILE_HPP
#include "Point.hpp"
#include "Solid.hpp"
#include "Disk.hpp"
#include <memory>
#include <random>
#include <string>
#include <list>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>
#include <iomanip>

class Environment;

//tests unitaires effectués

class Mobile{
    private : 
        double v0; // vitesse à l'origine
        double Dr; // diffusion rotationelle
        double Tau; // moyenne d'attente de l'évènement tumbling
        Point Coord; // coordonnées en repère cartésien dans R2, sert à calculer le déplacement total
        double Theta; // angle avec l'axe Ux
        int PoissonCount; // valeur du processus de comptage d'évènement "tumbling"
        double mu; //escape rate dans le cas d'une simulation en situation de non glissement (c'est en fait ici 1/mu)
        double time; // temps de simulation
        double Mt; // observation du déplacement total du processus au temps t
        bool IsCollided; // détection des collisions, processus aléatoire 
        friend void Solid::Coordoverwrite(Mobile & M, Point P);

    public : 
        Mobile(Point P = Point(0, 0), double v0 = 1, double Dr = 0, double Tau = 1, double Theta = 0, double mu = 0.5);
        Mobile(const Mobile & other);
        ~Mobile();
        Mobile operator=(const Mobile & other);
        Point getCoord() const; // autorise à utiliser Coord dans un calcul, pas à modifier la valeur d'un objet Mobile
        void simulation(Environment & E, double T=1, double h=0.01, Point X = Point(), uint64_t seed = 0, std::string Reorientation_mode = "isotropic");
        Solid* CollisionDetection(Environment & E);
        const double getMt() const;
        void simulation_expo(Environment & E, double T=1, double h=0.01, Point X = Point(), uint64_t seed = 0, std::string Reorientation_mode = "isotropic");
        void write_trajectory(Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename);
        Point& getCoord();
        double getV0ForDebug() const; 
        double getThetaForDebug() const;
        void test_exponentiality(Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename, int N_samples);
        void measure_diffusivity(Environment & E, double h, Point X, std::string Reorientation_mode, const std::string & filename,double time_upper_bound, int N_samples, int N_data);
        void diffusivity_function_of_tau(Environment & E, double h, Point X, std::string Reorientation_mode, const std::string & filename, double time_upper_bound);
        void measure_displacement(Environment & E, double T, double h, Point X, std::string Reorientation_mode, const std::string & filename, int N_samples);
    };


#endif //MOBILE_HPP