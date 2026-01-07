#include "tests.hpp"

# define M_PI           3.14159265358979323846  /* pi */
# define M_PI_2          1.57079632679           /*pi/2*/

using namespace std;


void test_IsCollided() {
    Disk d(Point(0,0), 1.0);        // disque unité
    Mobile m_inside(Point(0.5,0), 0,0,0,0);
    Mobile m_edge(Point(1.0,0), 0,0,0,0);
    Mobile m_out(Point(1.1,0), 0,0,0,0);

    assert( d.IsCollided(m_inside) == true  );  // strictement à l'intérieur
    assert( d.IsCollided(m_edge)   == true  );  // sur la frontière
    assert( d.IsCollided(m_out)    == false );  // légèrement à l'extérieur
}


void test_escapeAngle() {
    Disk d(Point(0,0), 1.0);
    Mobile m(Point(1,0),0,0,0,0); // mobile sur l'axe +x
    double* I = d.escapeAngle(m);
    double lower = I[0], upper = I[1];
    delete[] I;

    // Sur +x, la normale pointe vers +x : phi_n = 0
    // On attend [ -π/2 , +π/2 ] mais normalisé en [0, 2π] -> [ 3π/2 , π/2+2π ]
    double TWO_PI = 2*M_PI;
    double expected_lower = fmod(-M_PI_2 + TWO_PI, TWO_PI);  // ~4.712
    double expected_upper = fmod(M_PI_2 + TWO_PI, TWO_PI);   // ~1.571
    if (expected_lower > expected_upper) expected_upper += TWO_PI;

    assert( std::fabs(lower - expected_lower) < 1e-6 );
    assert( std::fabs(upper - expected_upper) < 1e-6 );

    // Test symétrique sur l'axe +y
    Mobile m2(Point(0,1),0,0,0,0);
    I = d.escapeAngle(m2);
    lower = I[0]; upper = I[1];
    delete[] I;

    // phi_n = π/2 => [0, π]
    assert( lower >= 0 && lower < upper );
    assert( std::fabs(lower - 0.0) < 1e-6 );
    assert( std::fabs(upper - M_PI) < 1e-6 );
}


void test_edgeProjection() {
    Disk d(Point(0,0), 1.0);
    Mobile m(Point(0.5,0),0,0,0,0);
    d.edgeProjection(m);
    double dist = std::hypot(m.getCoord().getx(), m.getCoord().gety());
    assert( std::fabs(dist - 1.0) < 1e-6 );  // mobile projeté sur le cercle

    // Test : mobile déjà sur le bord
    Mobile m2(Point(1.0,0),0,0,0,0);
    d.edgeProjection(m2);
    double dist2 = std::hypot(m2.getCoord().getx(), m2.getCoord().gety());
    assert( std::fabs(dist2 - 1.0) < 1e-6 ); // reste sur le bord
}


void TestDiskMethods() {
    std::cout << "\n========== TEST DISK METHODS ==========\n";

    
    std::vector<Disk> disks;
    disks.emplace_back(Point(0.0, 0.0), 1.0);  // Disque A au centre
    disks.emplace_back(Point(2.0, 0.0), 1.0);  // Disque B à droite
    disks.emplace_back(Point(-2.0, 1.0), 0.5); // Disque C plus petit

    
    std::vector<Mobile> mobiles;
    mobiles.emplace_back(Point(0.0, 0.0), 0,0,0,0);    // M0: au centre du disque A
    mobiles.emplace_back(Point(0.5, 0.0), 0,0,0,0);    // M1: à l'intérieur de A
    mobiles.emplace_back(Point(1.0, 0.0), 0,0,0,0);    // M2: sur la frontière de A
    mobiles.emplace_back(Point(1.5, 0.0), 0,0,0,0);    // M3: juste à l'extérieur de A
    mobiles.emplace_back(Point(2.4, 0.0), 0,0,0,0);    // M4: proche du disque B
    mobiles.emplace_back(Point(-2.0, 1.0),0,0,0,0);    // M5: au centre du disque C

    
    for (size_t d = 0; d < disks.size(); ++d) {
        std::cout << "\n==== Disque #" << d << " ====\n";

        for (size_t m = 0; m < mobiles.size(); ++m) {
            std::cout << "\n--- Mobile #" << m
                      << " position=(" << mobiles[m].getCoord().getx() << ", "
                      << mobiles[m].getCoord().gety() << ") ---\n";

            
            bool collision = disks[d].IsCollided(mobiles[m]);
            std::cout << "[IsCollided] Résultat = " << (collision ? "OUI" : "NON") << "\n";

            
            if (collision) {
                double* interval = disks[d].escapeAngle(mobiles[m]);
                std::cout << "[escapeAngle] Intervalle d'échappement [lower, upper] = ["
                          << interval[0] << ", " << interval[1] << "] radians\n";
                delete[] interval;

                
                Mobile tmp = mobiles[m];
                std::cout << "[edgeProjection] Projection du mobile sur la frontière...\n";
                disks[d].edgeProjection(tmp);
                std::cout << "Nouvelle position mobile = ("
                          << tmp.getCoord().getx() << ", "
                          << tmp.getCoord().gety() << ")\n";
            } else {
                std::cout << "Pas de collision => escapeAngle / edgeProjection non testés.\n";
            }
        }
    }
}

void advance_until_contact_and_print(Mobile & m, Environment & E, double h, int max_steps, const std::string & tag) {
    std::cout << "\n--- Test: " << tag << " ---\n";
    std::cout << "Start position = (" << m.getCoord().getx() << ", " << m.getCoord().gety()
              << "), Theta = " << m.getThetaForDebug() /* see note below */ << "\n";

    Solid* collision = nullptr;
    int step = 0;
    for (; step < max_steps; ++step) {
        // test collision at current position BEFORE advancing
        collision = m.CollisionDetection(E);
        if (collision != nullptr) {
            std::cout << "[info] Collision detected BEFORE moving (step=" << step << ")\n";
            break;
        }

        // advance one time-step h in direction Theta
        double dx = m.getV0ForDebug() * h * std::cos(m.getThetaForDebug());
        double dy = m.getV0ForDebug() * h * std::sin(m.getThetaForDebug());
        m.getCoord() = m.getCoord() + Point(dx, dy);

        // check collision AFTER moving
        collision = m.CollisionDetection(E);
        if (collision != nullptr) {
            std::cout << "[info] Collision detected AFTER moving (step=" << step << ")\n";
            break;
        }
    }

    if (collision == nullptr) {
        std::cout << "No collision within " << max_steps << " steps.\n";
        return;
    }

    // Project onto the boundary (so mobile sits exactly on contact point)
    collision->edgeProjection(m);

    // Get the escape-interval and print it
    double* interval = collision->escapeAngle(m);
    std::cout << "Contact position (after projection) = (" << m.getCoord().getx() << ", " << m.getCoord().gety() << ")\n";
    std::cout << "escapeAngle interval = [lower = " << interval[0] << ", upper = " << interval[1] << "] (radians)\n";

    delete[] interval;
}

/*
// ======= environnement avec un disque centré en (0,0), rayon 1.0 ========
    Environment E; // constructeur default -> vide
    E.get_Solid_vector().push_back(std::make_unique<Disk>(Point(0.0, 0.0), 1.0));

    // paramètres simulation pour test (pas de tumble en fixant Tau = 0)
    double v0 = 1.0;
    double Dr = 0.0;
    double Tau = 0.0;      // IMPORTANT : Tau = 0 => probabilite de tumble h*Tau = 0
    double h = 0.01;
    int max_steps = 2000;

    // === Mobile A : arrive horizontalement depuis x>1 vers (1,0) ===
    Point startA(2.0, 0.0);
    double thetaA = M_PI; // direction vers la gauche (-> collision sur (1,0))
    Mobile mA(startA, v0, Dr, Tau, thetaA);

    // === Mobile B : arrive verticalement depuis au-dessus vers (1,0) ===
    Point startB(1.0, 2.0);
    double thetaB = -M_PI_2; // direction vers le bas
    Mobile mB(startB, v0, Dr, Tau, thetaB);

    // === Mobile C : arrive verticalement depuis en-dessous vers (1,0) ===
    Point startC(1.0, -2.0);
    double thetaC = M_PI_2; // direction vers le haut
    Mobile mC(startC, v0, Dr, Tau, thetaC);

    // Avance chacun jusqu'au contact et affiche l'intervalle
    advance_until_contact_and_print(mA, E, h, max_steps, "Mobile A (from right)");
    advance_until_contact_and_print(mB, E, h, max_steps, "Mobile B (from top)");
    advance_until_contact_and_print(mC, E, h, max_steps, "Mobile C (from bottom)");

    std::cout << "\nFIN tests.\n";
*/



/*
Point P(0, 0);
Environment E("uniform_Disks", 1, 0.4);
Mobile M(P, 1, 0, 1, 0, 0.5); // v0=10, Dr=0, Tau=1, Theta=0, mu = 0.7
========== Resultat ==========
Tau* Ôëê 86.3192
D* Ôëê 1.99505
Nombre dÔÇÖiterations : 10
Duree totale : 6265.37 s
==============================






*/


//-----------------main de calcul des maximums de diffusivité + simulation et écriture de coordonnées---------------//
/*
int main(){
      
    
    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(std::random_device{}());
    
    Point P(0, 0);
    Environment E_empty("empty");

    Environment E1("uniform_Disks", 1, 0.4); // large uniform Disks
    Environment E2("uniform_Disks", 1, 0.2); // small uniform Disks
    Environment E3("uniform_Squares", 1, 0.4); // large uniform Squares
    Environment E4("uniform_Squares", 1, 0.2); // small uniform Squares

    Mobile M(P, 1, 5, 1, 0, 0.5); // v0=1, Dr=0, Tau=1, Theta=0, mu = 0.7
    M.write_trajectory(E1, 100, 0.01, P, seed, "isotropic", "C:\\Users\\Jules\\Desktop\\Stage ENS 4A\\backup1_simu_stage_4A\\data\\Trajectories.csv");
    

    //M.write_trajectory_expo(E3, 100, 0.01, P, seed, "isotropic", "C:\\Users\\Jules\\Desktop\\Stage ENS 4A\\data\\Trajectories.csv");
    
    // large uniform Disks
    M.diffusivity_function_of_tau(E1, P, "isotropic", "Diffusivity_IS_large_uniform_Disks.csv", 100, 1000, 20);
    M.diffusivity_function_of_tau(E1, P, "run_and_reverse", "Diffusivity_RR_large_uniform_Disks.csv", 100, 1000, 20);
    // small uniform Disks
    M.diffusivity_function_of_tau(E2, P, "isotropic", "Diffusivity_IS_small_uniform_Disks.csv", 100, 1000, 20);
    M.diffusivity_function_of_tau(E2, P, "run_and_reverse", "Diffusivity_RR_small_uniform_Disks.csv", 100, 1000, 20);
    
    
    // large uniform Squares
    //M.diffusivity_function_of_tau(E3, P, "isotropic", "Diffusivity_IS_large_uniform_Squares.csv", 10, 1000, 10, 0.82690);
    //M.diffusivity_function_of_tau(E3, P, "run_and_reverse", "Diffusivity_RR_large_uniform_Squares.csv", 100, 1000, 20);
    // small uniform Squares
    //M.diffusivity_function_of_tau(E4, P, "isotropic", "Diffusivity_IS_small_uniform_Squares.csv", 200, 1000, 20, 0.82690);
    //M.diffusivity_function_of_tau(E4, P, "run_and_reverse", "Diffusivity_RR_small_uniform_Squares.csv", 1000, 20, 0.60555);
    
    return 0; 
}


*/