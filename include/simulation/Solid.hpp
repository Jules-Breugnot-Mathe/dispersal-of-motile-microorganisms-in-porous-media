#ifndef SOLID_HPP
#define SOLID_HPP

#include "Point.hpp"

class Mobile; // forward declaration pour éviter les inclusions circulaires

//tests unitaires effectués

class Solid{
    private : 
        int id; 
    public : 
        Solid();
        ~Solid();
        virtual bool IsCollided(const Mobile & M)=0; // detection de collision
        const int getid() const;
        int & getid();
        virtual double* escapeAngle(const Mobile & M)=0; // calcul de l'intervalle pour l'angle de sortie
        virtual void edgeProjection(Mobile & M)=0; // projection d'un mobile sur la frontière du solide
        void Coordoverwrite(Mobile & M, Point P);
};

/*
// Création d'un vecteur de unique_ptr<Solid>
    std::vector<std::unique_ptr<Solid>> solids;

    // Instanciation de rectangles et disques
    solids.push_back(std::make_unique<Rectangle>(Point(0,0), Point(1,0), Point(1,1), Point(0,1)));
    solids.push_back(std::make_unique<Disk>(Point(2,2), 1.0));
    solids.push_back(std::make_unique<Rectangle>()); // avec points par défaut
    solids.push_back(std::make_unique<Disk>());      // disque par défaut

    // Création d'un Mobile pour tester IsCollided
    Mobile mob;

    // Boucle sur tous les solides et appel de IsCollided via le type Solid
    for (size_t i = 0; i < solids.size(); ++i) {
        bool collided = solids[i]->IsCollided(mob);
        std::cout << "Solide #" << i << " collision = " << collided << std::endl;
*/


#endif // SOLID_HPP

