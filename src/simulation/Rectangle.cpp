#include "Rectangle.hpp"
#include <cmath>
#include <limits>

#define M_PI           3.14159265358979323846
#define M_PI_2         1.57079632679489661923

const double tolerance = 1e-9;

Rectangle::Rectangle(Point p1, Point p2, Point p3, Point p4){
    vertex[0] = p1, vertex[1] = p2; vertex[2] = p3; vertex[3] = p4;
    axis[0] = p1 - p4; axis[1] = p2 - p1; axis[2] = p3 - p2; axis[3] = p4 - p3;
}

Rectangle::Rectangle(Point center, double radius) {
    // Carré de côté 2*radius, centré en center, aligné sur les axes
    vertex[0] = Point(center.getx() - radius, center.gety() - radius); // bas gauche
    vertex[1] = Point(center.getx() + radius, center.gety() - radius); // bas droit
    vertex[2] = Point(center.getx() + radius, center.gety() + radius); // haut droit
    vertex[3] = Point(center.getx() - radius, center.gety() + radius); // haut gauche

    axis[0] = vertex[1] - vertex[0];
    axis[1] = vertex[2] - vertex[1];
    axis[2] = vertex[3] - vertex[2];
    axis[3] = vertex[0] - vertex[3];
}

Rectangle::~Rectangle(){}

Rectangle::Rectangle(const Rectangle & other){
    for (int j=0;j<4;j++){
        this->vertex[j] = other.vertex[j];
        this->axis[j] = other.axis[j];
    }
}

Rectangle Rectangle::operator=(const Rectangle & other){
    for (int j=0;j<4;j++){
        this->vertex[j] = other.vertex[j];
        this->axis[j] = other.axis[j];
    }
    return *this;
}

std::ostream& operator<<(std::ostream& o, const Rectangle & rec){
    o << rec.vertex[0] << rec.vertex[1] << rec.vertex[2] << rec.vertex[3] << std::endl;
    return o;
}

static inline double distance_point_segment(const Point& P, const Point& A, const Point& B, Point& closest)
{
    Point AB = B - A;
    Point AP = P - A;
    double ab2 = AB.dot_product(AB);
    double t = (ab2 > 0.0) ? (AP.dot_product(AB) / ab2) : 0.0;
    t = std::max(0.0, std::min(1.0, t)); // Clamp t in [0,1]
    closest = A + AB * t;
    Point diff = P - closest;
    return std::sqrt(diff.dot_product(diff));
}

bool Rectangle::IsCollided(const Mobile & M)
{
    Point P = M.getCoord();
    bool inside = true;

    for (int i = 0; i < 4; ++i) {
        Point A = vertex[i];
        Point B = vertex[(i + 1) % 4];
        Point edge = B - A;
        Point toPoint = P - A;
        double cross = edge.getx() * toPoint.gety() - edge.gety() * toPoint.getx();

        // Sommets dans le sens anti-horaire :
        if (cross < -tolerance) {
            inside = false;
            break;
        }
    }

    return inside;
}

double* Rectangle::escapeAngle(const Mobile & M)
{
    double* I = new double[2];

    Point P = M.getCoord();
    double minDist = std::numeric_limits<double>::max();
    Point closestPoint;
    Point normal(0,0);

    // Trouver l'arête la plus proche
    for (int i = 0; i < 4; ++i) {
        Point A = vertex[i];
        Point B = vertex[(i + 1) % 4];
        Point C;
        double d = distance_point_segment(P, A, B, C);
        if (d < minDist) {
            minDist = d;
            closestPoint = C;

            // Normale sortante (perpendiculaire à l’arête, dirigée vers l’extérieur)
            Point edge = B - A;
            Point outward(-edge.gety(), edge.getx()); // rotation +90°
            
            // Vérifie le bon sens de la normale (doit pointer vers P si P est à l’extérieur)
            Point toP = P - C;
            if (outward.dot_product(toP) < 0) {
                outward = outward * (-1);
            }

            // Normalisation
            double norm = std::sqrt(outward.dot_product(outward));
            normal = outward * (1.0 / norm);
        }
    }
    normal = normal*(-1);
    // Angle de la normale sortante
    double phi_n = std::atan2(normal.gety(), normal.getx());
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


void Rectangle::edgeProjection(Mobile & M)
{
    Point P = M.getCoord();
    double minDist = std::numeric_limits<double>::max();
    Point closestPoint;
    Point normal(0,0);

    // Trouver le bord le plus proche et sa normale sortante
    for (int i = 0; i < 4; ++i) {
        Point A = vertex[i];
        Point B = vertex[(i + 1) % 4];
        Point C;
        double d = distance_point_segment(P, A, B, C);
        if (d < minDist) {
            minDist = d;
            closestPoint = C;

            Point edge = B - A;
            Point outward(-edge.gety(), edge.getx());
            Point toP = P - C;
            if (outward.dot_product(toP) < 0) outward = outward * (-1);
            double norm = std::sqrt(outward.dot_product(outward));
            normal = outward * (1.0 / norm);
        }
    }

    // On repousse le point juste à la frontière
    Point M_proj = P - normal * (minDist - tolerance);
    this->Coordoverwrite(M, M_proj);
}



const Point& Rectangle::getVertex(int i) const
{
    return vertex[i % 4];
}
