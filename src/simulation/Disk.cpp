#include "Disk.hpp"
#include <cmath>

# define M_PI           3.14159265358979323846  /* pi */
# define M_PI_2          1.57079632679           /*pi/2*/
const double tolerance = 1e-9;

Disk::Disk(Point center, double radius){
    this->center = center;
    this->radius = radius;
}

Disk::~Disk(){
}

Disk::Disk(const Disk & other){
    this->center = other.center;
    this->radius = other.radius;
}

Disk Disk::operator=(const Disk & other){
    this->center = other.center;
    this->radius = other.radius;
    return *this;
}


bool Disk::IsCollided(const Mobile & M){
    double X = center.getx()-M.getCoord().getx(); double Y = center.gety() - M.getCoord().gety();
    if (std::sqrt(X*X + Y*Y) <= radius + tolerance) {
        return 1;
    }
    return 0;
}



double* Disk::escapeAngle(const Mobile & M) {
    double* I = new double[2];

    double Cx = this->center.getx();
    double Cy = this->center.gety();
    double x  = M.getCoord().getx();
    double y  = M.getCoord().gety();
    double phi_n = std::atan2(y - Cy, x - Cx);
    const double TWO_PI = 2.0 * M_PI;
    phi_n = std::fmod(phi_n, TWO_PI);
    if (phi_n < 0) phi_n += TWO_PI;
    double lower = phi_n - M_PI_2;
    double upper = phi_n + M_PI_2;
    lower = std::fmod(lower, TWO_PI);
    if (lower < 0) lower += TWO_PI;
    upper = std::fmod(upper, TWO_PI);
    if (upper < 0) upper += TWO_PI;
    if (lower > upper) upper += TWO_PI;
    I[0] = lower;
    I[1] = upper;
    return I;
}



void Disk::edgeProjection(Mobile & M){


    //std::cout<<"centre du disque : "<<this->center<<std::endl;
    //std::cout<<"rayon du disque : "<<this->radius<<std::endl;
    
    Point v = (this->center)*(-1) + M.getCoord(); // modifie les coordonnees!!!!!!!
    //std::cout<<"coordonnees du mobile avant projection : "<<M.getCoord()<<std::endl;

    //std::cout<<"vecteur v  : "<<v<<std::endl; // pointe en direction du mobile
    double dist = std::sqrt(v.getx()*v.getx() + v.gety()*v.gety());
    //std::cout<<"distance Mobile-centre: "<<dist<<std::endl;
    v = v*(1/(std::sqrt(v.getx()*v.getx() + v.gety()*v.gety())));
    //std::cout<<"vecteur normalise v : "<<v<<std::endl; // vecteur normalisé qui pointe vers le mobile
    double deplacement = this->radius - dist;
    //std::cout<<"deplacement a faire : "<<deplacement<<std::endl;
    Point deplacement_vectoriel = v*(deplacement);
    //std::cout<<"deplacement vectoriel a faire : "<<deplacement_vectoriel<<std::endl;

    
    Point M_proj = M.getCoord() + deplacement_vectoriel;
    //std::cout<<"M projete : "<<M_proj<<std::endl;
    this->Coordoverwrite(M, M_proj);
}



const Point& Disk::getCenter() const {
    return center;
}

double Disk::getRadius() const { 
    return radius;
}