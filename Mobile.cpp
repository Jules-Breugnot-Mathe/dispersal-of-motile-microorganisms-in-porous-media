#include "Mobile.hpp"
#include "Environment.hpp"

#include <chrono>

# define M_PI           3.14159265358979323846  /* pi */

Mobile::Mobile(Point P, double v0, double Dr, double Tau, double Theta, double mu){
    this->v0 = v0; this->Dr = Dr; this->Tau = Tau; this->Theta = Theta; this->Coord = P; this->mu = mu;
    this->Free_coord = P; this->Loop = {0, 0};
}

Mobile::~Mobile(){}

Mobile::Mobile(const Mobile & other){
    this->Coord = other.Coord; this->Dr = other.Dr; this->Tau = other.Tau;
    this->Theta = other.Theta; this->v0 = other.v0; 
}

Mobile Mobile::operator=(const Mobile & other){
    this->Coord = other.Coord; this->Dr = other.Dr; this->Tau = other.Tau;
    this->Theta = other.Theta; this->v0 = other.v0; 
    return *this;
}

Point Mobile::getCoord() const{
    return this->Coord;
}

Point Mobile::getFreeCoord() const {
    return this->Free_coord;
}

const double Mobile::getMt() const{
    return this->Mt;
}

Point& Mobile::getCoord(){
    return this->Coord;
}

Point& Mobile::getFreeCoord(){
    return this->Free_coord;
}

double Mobile::getV0ForDebug() const {
    return v0; 
}
double Mobile::getThetaForDebug() const {
    return Theta;
}


Solid* Mobile::CollisionDetection(const Environment & E){ 
    //On parcours l'ensemble des corps de l'environnement à l'aide d'un itérateur, 
    //et on renvoie en sortie un pointeur de base sur le corps en collision,
    //ou le pointeur nul si aucune collision
    //std::cout<<"adresse de E : "<<&E<<std::endl;
    std::vector<std::unique_ptr<Solid>>::const_iterator it;
    
    for (it=E.get_Solid_vector().begin(); it != E.get_Solid_vector().end(); it++){
        if ((*it)->IsCollided(*this)){ // on déréférence l'itérateur pour avoir un pointeur sur Solid
            return (*it).get(); // on renvoie un pointeur de base sur Solid
        }
    }
    return nullptr;  // en absence de collision on renvoie un pointeur nul
}



void Mobile::simulation(const Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode){
    // temps courant, avant l'execution
    std::chrono::high_resolution_clock::time_point a= std::chrono::high_resolution_clock::now();

    double N = T/h;
    this->Coord = X;
    std::mt19937_64 engine(seed);
    std::bernoulli_distribution tumble(h/this->Tau); // variable de bernoulli de l'evènement "tumbling"
    std::bernoulli_distribution escape(this->mu); // variable de bernoulli de l'évènement "escape"
    std::uniform_real_distribution<double> theta(0, 2*M_PI); // variable d'angle de réorientation 
    std::uniform_real_distribution<double> U(0, 1);
    bool tumble_outcome; // observation de la variable tumbling
    bool escape_outcome; // observation de la variable escape
    double phi_outcome; // observation de la variable de réorientation après collision
    double time_of_first_collision; // temps d'attente de la première collision

    Solid* collision = nullptr; // on initialise la collision
    this->Theta = theta(engine); // on initialise l'orientation aléatoire

    for (int j=0;j<N;j++){ //on parcours le temps de 0 à T = Nh

        E.getDomain().MakeLoop(*this);
        //Coord est dans le domaine, Loop a compté le nombre de tours, et Free_coord est inchangé et encapsule les coordonnées "réelles"

        tumble_outcome = tumble(engine); //on observe la variable tumble
        collision = this->CollisionDetection(E); // on vérifie la collision
        if (collision == nullptr) {
            // pas de collision détectée 
            if (tumble_outcome == 1) {
                // on doit se réorienter suivant le mode de réorientation
                if (Reorientation_mode == "run_and_reverse") {
                    double X = 2*std::asin(2*U(engine) - 1);
                    this->Theta = this->Theta + M_PI/2 + X;
                }
                else if (Reorientation_mode == "isotropic") {
                    this->Theta = theta(engine);
                }
            }
            else {
                // pas de tumble : on avance sans contrainte
            }
            Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // on avance
        }
        else {
            // collision détectée 
            // il faut projeter la particule sur la frontière du solide pour ne pas créer d'erreur avec le calcul d'angle 
            collision->edgeProjection(*this);
            escape_outcome = escape(engine); // on observe la variable escape (indépendance?)
           if ((tumble_outcome == 1)&&(escape_outcome == 1)) {
                // le mobile s'échappe : on calcule l'intervalle dans lequel simuler une variable d'angle uniforme (angle de sortie) 
                std::uniform_real_distribution<double> phi(collision->escapeAngle(*this)[0], collision->escapeAngle(*this)[1]); 
                // variable d'angle de réorientation. Collision pointe sur un Solid, on appelle la méthode virtuelle pure escapeAngle,
                //qui se spécifie selon le solide, puis on en tire les bornes inf et sup pour l'angle.
                phi_outcome = phi(engine);
                this->Theta = phi_outcome; // on se réoriente
                Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // et on avance
            }
            else {
                // le mobile ne bouge pas
            }
        }
    }
    this->Mt = std::sqrt((this->getFreeCoord().getx() - X.getx())*(this->getFreeCoord().getx() - X.getx()) + (this->getFreeCoord().gety() - X.gety())*(this->getFreeCoord().gety() - X.gety()));
    // On a calculé le déplacement total observé Mt
    
    // temps courant, apres l'execution
    std::chrono::high_resolution_clock::time_point b= std::chrono::high_resolution_clock::now();
    // mesurer la difference, et l'exprimer en microsecondes 
    unsigned int time= std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    //std::cout<<"simulation d'une trajectoire sur un temps T = "<<T<<" : "<<time*0.000001 <<" s"<<std::endl; // temps affiché en secondes

    return;
}






void Mobile::simulation_expo(Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode){
    double N = T/h;
    this->Coord = X;
    std::mt19937_64 engine(seed);
    std::exponential_distribution<double> Ti(1/this->Tau);
    std::bernoulli_distribution escape(this->mu);
    std::uniform_real_distribution<double> theta(0, 2*M_PI); // variable d'angle de réorientation 
    Solid* collision = nullptr; // on initialise la collision
    this->Theta = theta(engine); // on initialise l'angle de départ aléatoire
    // on a d'abord besoin de simuler les temps d'attente de tumbling
    std::list<std::pair<double, double>> random_events;  // liste de temps d'attentes Ti suivant une loi exponentielle, avec escape booléen
    double T_outcome = 0.0; int discrete_T_outcome; //temps exponentiels et versions discrètes
    double T_sum = 0.0;
    double* bounds = nullptr;

    while (true) {
        T_outcome = Ti(engine);
        if (T_sum + T_outcome > T) break;   
        T_sum += T_outcome;
        random_events.push_front({std::floor(T_outcome/h), escape(engine)}); // source d'erreur : approximation des temps exponentiels
        //pour coller avec le pas de discrétisation, doit tendre vers 0 avec h (calculer ordre de grandeur de l'erreur)
    }

    random_events.push_front({0, 1}); // on insère un temps d'attente nul au début pour pouvoir décrémenter it sans problème
    random_events.push_back({T-T_sum, 1}); // on insère un dernier temps d'attente à la fin pour finir le processus

    std::list<std::pair<double, double>>::iterator it = random_events.begin();
    ++it; // on fait pointer l'itérateur sur le deuxième élément de la liste random_events

    for (; it != random_events.end(); it++){ //on va parcourir les temps d'attente exponentiels
        collision = this->CollisionDetection(E);
        if ((collision == nullptr)||(std::prev(it)->second == 1)){ // pas de collison ou dernier escape positif
            std::cout<<"it->first : "<<it->first<<std::endl;
            std::cout<<"it->second : "<<it->second<<std::endl;
            for (int j=0 ; j < it->first ; j++){ // on va avancer
                Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // on avance
                collision = this->CollisionDetection(E);
                if (collision != nullptr) {
                    std::cout<<"collision !"<<std::endl;
                    collision->edgeProjection(*this); // on projette le mobile sur la frontière de l'obstacle
                    bounds = collision->escapeAngle(*this);
                    std::uniform_real_distribution<double> alpha(bounds[0], bounds[1]);
                    this->Theta = alpha(engine);
                    break; // on revient à l'itérateur sur random_events
                }
                // en l'absence de collision, on se réoriente de facon aléatoire 
                else {
                    if (Reorientation_mode == "isotropic") {
                        //on choisit un angle dans l'intervalle [0, 2*pi]
                        this->Theta = theta(engine);
                    }
                    else if (Reorientation_mode == "run_and_reverse"){
                        //on choisit un angle centré sur l'angle opposé à theta actuel
                    }
                }
            }
        }
    }
    delete[] bounds;
    this->Mt = std::sqrt((this->getCoord().getx() - X.getx())*(this->getCoord().getx() - X.getx()) + (this->getCoord().gety() - X.gety())*(this->getCoord().gety() - X.gety()));
    // On a calculé le déplacement total observé Mt

}
 



/*
    //-------------TEST DE SIMULATION--------------//
    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(std::random_device{}());
    
    Point P(0, 0);
    Point P1(1, 0), P2(0, 1), P3(-1, 0), P4(-1, -1);
    double R = std::sqrt(2)/2; 
    Environment E("uniform_Disks", 10);
    Mobile M(P, 1, 1, 1, 0);
    M.simulation(E, 1000, 0.01, P, seed);
    cout<<"Coordonnees finales : "<<M.getCoord()<<endl;  
    cout<<"Deplacement Total : "<<M.getMt()<<endl; 
*/

/*
    //-----------------TEST de simulation avec variables exponentielles------------------//
    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(std::random_device{}());
    Point P(0, 0);
    Point P1(1, 0), P2(0, 1), P3(-1, 0), P4(-1, -1);
    double R = std::sqrt(2)/2; 
    Environment E("uniform_Disks", 10);
    Mobile M(P, 1, 1, 1, 0);
    M.simulation_expo(E, 10, 0.01, P, seed, "isotropic");
    cout<<"Coordonnees finales : "<<M.getCoord()<<endl;  
    cout<<"Deplacement Total : "<<M.getMt()<<endl;
*/


void Mobile::write_trajectory(const Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename){
    // temps courant, avant l'execution
    std::chrono::high_resolution_clock::time_point a= std::chrono::high_resolution_clock::now();

    std::ofstream ofs;
    ofs.open(filename, std::ofstream::out | std::ofstream::trunc);
    if (ofs.is_open() == false) {
        std::cerr<<"Mobile::write_trajectory : impossible d'ouvrir le fichier"<<std::endl;
        return ;
    }
    //on copie le corps de Mobile::simulation et on rajoute l'écriture des coordonnées à chaque incrément de temps
    double N = T/h;
    this->Coord = X;
    std::mt19937_64 engine(seed);
    std::bernoulli_distribution tumble(h/this->Tau); // variable de bernoulli de l'evènement "tumbling"
    std::bernoulli_distribution escape(this->mu); // variable de bernoulli de l'évènement "escape"
    std::uniform_real_distribution<double> theta(0, 2*M_PI); // variable d'angle de réorientation 
    std::uniform_real_distribution<double> U(0, 1);
    bool tumble_outcome; // observation de la variable tumbling
    bool escape_outcome; // observation de la variable escape
    double phi_outcome; // observation de la variable de réorientation après collision
    double time_of_first_collision; // temps d'attente de la première collision

    Solid* collision = nullptr; // on initialise la collision
    this->Theta = theta(engine); // on initialise l'orientation aléatoire

    for (int j=0;j<N;j++){ //on parcours le temps de 0 à T = Nh

        E.getDomain().MakeLoop(*this);
        //Coord est dans le domaine, Loop a compté le nombre de tours, et Free_coord est inchangé et encapsule les coordonnées "réelles"

        ofs <<this->Free_coord.getx()<<","<<this->Free_coord.gety()<<std::endl; // on écrit les coordonnées dans le fichier

        tumble_outcome = tumble(engine); //on observe la variable tumble
        collision = this->CollisionDetection(E); // on vérifie la collision
        if (collision == nullptr) {
            // pas de collision détectée 
            if (tumble_outcome == 1) {
                // on doit se réorienter suivant le mode de réorientation
                if (Reorientation_mode == "run_and_reverse") {
                    double X = 2*std::asin(2*U(engine) - 1);
                    this->Theta = this->Theta + M_PI/2 + X;
                }
                else if (Reorientation_mode == "isotropic") {
                    this->Theta = theta(engine);
                }
            }
            else {
                // pas de tumble : on avance sans contrainte
            }
            Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // on avance
        }
        else {
            // collision détectée 
            // il faut projeter la particule sur la frontière du solide pour ne pas créer d'erreur avec le calcul d'angle 
            collision->edgeProjection(*this);
            escape_outcome = escape(engine); // on observe la variable escape (indépendance?)
           if ((tumble_outcome == 1)&&(escape_outcome == 1)) {
                // le mobile s'échappe : on calcule l'intervalle dans lequel simuler une variable d'angle uniforme (angle de sortie) 
                std::uniform_real_distribution<double> phi(collision->escapeAngle(*this)[0], collision->escapeAngle(*this)[1]); 
                // variable d'angle de réorientation. Collision pointe sur un Solid, on appelle la méthode virtuelle pure escapeAngle,
                //qui se spécifie selon le solide, puis on en tire les bornes inf et sup pour l'angle.
                phi_outcome = phi(engine);
                this->Theta = phi_outcome; // on se réoriente
                Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // et on avance
            }
            else {
                // le mobile ne bouge pas
            }
        }
    }
    this->Mt = std::sqrt((this->getFreeCoord().getx() - X.getx())*(this->getFreeCoord().getx() - X.getx()) + (this->getFreeCoord().gety() - X.gety())*(this->getFreeCoord().gety() - X.gety()));
    // On a calculé le déplacement total observé Mt
    ofs.close();
    
    // temps courant, apres l'execution
    std::chrono::high_resolution_clock::time_point b= std::chrono::high_resolution_clock::now();
    // mesurer la difference, et l'exprimer en microsecondes 
    unsigned int time= std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    std::cout<<"ecriture d'une trajectoire sur un temps T = "<<T<<" : "<<time*0.000001 <<" s"<<std::endl; // temps affiché en secondes

    return;
}


void Mobile::test_exponentiality(Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename, int N_samples){
    std::ofstream ofs;
    ofs.open(filename, std::ofstream::out | std::ofstream::trunc);
    if (ofs.is_open() == false) {
        std::cerr<<"Mobile::test_exponentiality : impossible d'ouvrir le fichier"<<std::endl;
        return ;
    }
    //------------------------- variables pour le test d'exponentialité du temps d'attente de collision-------------------------//
    double time_before_collision = 0; // va servir à observer le temps d'attente avant prochaine collision
    int collision_clock = 0; // incrément de temps discret servant à observer le temps d'attente avant prochaine collision
    //-------------------------------------------------------------------------------------------------------------------------//
    double N = T/h;
    this->Coord = X;
    std::mt19937_64 engine(seed);
    std::bernoulli_distribution tumble(h/this->Tau); // variable de bernoulli de l'evènement "tumbling"
    std::bernoulli_distribution escape(this->mu); // variable de bernoulli de l'évènement "escape"
    std::uniform_real_distribution<double> theta(0, 2*M_PI); // variable d'angle de réorientation 
    std::uniform_real_distribution<double> U(0, 1);
    bool tumble_outcome; // observation de la variable tumbling
    bool escape_outcome; // observation de la variable escape
    double phi_outcome; // observation de la variable de réorientation après collision
    double time_of_first_collision; // temps d'attente de la première collision

    Solid* collision = nullptr; // on initialise la collision
    this->Theta = theta(engine); // on initialise l'orientation aléatoire

    for (int k=0 ; k<N_samples ; k++) { // nombre d'echantillons

        time_before_collision = 0;
        collision_clock = 0;

        for (int j=0;j<N;j++){ //on parcours le temps de 0 à T = Nh
        collision_clock++; // on incrémente le compteur de temps avant collision

        tumble_outcome = tumble(engine); //on observe la variable tumble
        collision = this->CollisionDetection(E); // on vérifie la collision
        if (collision == nullptr) {
            // pas de collision détectée 
            if (tumble_outcome == 1) {
                // on doit se réorienter suivant le mode de réorientation
                if (Reorientation_mode == "run_and_reverse") {
                    double X = 2*std::asin(2*U(engine) - 1);
                    this->Theta = this->Theta + M_PI/2 + X;
                }
                else if (Reorientation_mode == "isotropic") {
                    this->Theta = theta(engine);
                }
            }
            else {
                // pas de tumble : on avance sans contrainte
            }
            Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // on avance
        }
        else {
            // collision détectée 
            if (collision_clock != 1) {
                time_before_collision = h*collision_clock;
                ofs << time_before_collision <<" , ";
            }
            collision_clock = 0;
            // il faut projeter la particule sur la frontière du solide pour ne pas créer d'erreur avec le calcul d'angle 
            collision->edgeProjection(*this);
            escape_outcome = escape(engine); // on observe la variable escape (indépendance?)
           if ((tumble_outcome == 1)&&(escape_outcome == 1)) {
                // le mobile s'échappe : on calcule l'intervalle dans lequel simuler une variable d'angle uniforme (angle de sortie) 
                std::uniform_real_distribution<double> phi(collision->escapeAngle(*this)[0], collision->escapeAngle(*this)[1]); 
                // variable d'angle de réorientation. Collision pointe sur un Solid, on appelle la méthode virtuelle pure escapeAngle,
                //qui se spécifie selon le solide, puis on en tire les bornes inf et sup pour l'angle.
                phi_outcome = phi(engine);
                this->Theta = phi_outcome; // on se réoriente
                Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // et on avance
            }
            else {
                // le mobile ne bouge pas
            }
        }
    }
    
    ofs << std::endl;

    }
    this->Mt = std::sqrt((this->getCoord().getx() - X.getx())*(this->getCoord().getx() - X.getx()) + (this->getCoord().gety() - X.gety())*(this->getCoord().gety() - X.gety()));
    std::cout<<"Distance finale : "<<Mt<<std::endl;

    ofs.close();
    return;
}


void Mobile::measure_diffusivity(Environment & E, double h, Point X,
                                 std::string Reorientation_mode,
                                 const std::string & filename,
                                 double time_upper_bound, int N_samples, int N_data)
{
    /*
    on va stocker dans un fichier csv de N_samples lignes et de N_data colonnes les observations (samples) de D à différents
    temps de simulation (il y en a donc N_data). D est définis comme la valeur Mt^2 renormalisée par 2dt. Le but est d'observer numériquement
    à partir de quel temps de simulation D converge à moins de 5% d'erreur, comparé à la valeur théorique
    obtenu par l'équivalent multivarié du théorème de berry-esseen, dans le cas particulier du déplacement libre dans R2.
    On cherche également à savoir si le temps de convergence de D dans le cas non trivial est le même, et à mesurer D en différents milieux
    */

    std::chrono::high_resolution_clock::time_point a = std::chrono::high_resolution_clock::now();

    if ((std::abs(X.getx()) >= E.getDomain().getradius()) || (std::abs(X.gety()) >= E.getDomain().getradius())) {
        std::cerr << "measure_diffusivity : le point de départ des simulations doit être dans le domaine de l'environnement" << std::endl;
        return;
    }

    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "measure_diffusivity : impossible d'ouvrir le fichier\n";
        return;
    }

    double T = time_upper_bound;       // durée totale de la simulation
    int N_tot = static_cast<int>(T / h);  // nombre total d’itérations
    int N_interval = static_cast<int>(std::floor(N_tot / N_data));      // nombre d’itérations entre deux enregistrements de D

    // Écriture de la première ligne contenant les temps (T, 2T, ..., N_data*T)
    for (int i = 0; i < N_data; ++i) {
        double t_i = (i + 1) * (T / N_data);
        ofs << t_i;
        if (i < N_data - 1) ofs << ",";
    }
    ofs << "\n";

    for (int j = 0; j < N_samples; ++j) {
        // Initialisation du mobile pour ce sample
        this->Coord = X;
        this->Free_coord = X;
        this->Loop = {0, 0};
        this->Theta = 0.0;
        this->Mt = 0.0;

        uint64_t seed = static_cast<uint64_t>(std::random_device{}());
        std::mt19937_64 engine(seed);
        std::bernoulli_distribution tumble(h / this->Tau);
        std::bernoulli_distribution escape(this->mu);
        std::uniform_real_distribution<double> theta(0, 2 * M_PI);
        std::uniform_real_distribution<double> U(0, 1);

        Solid* collision = nullptr;
        this->Theta = theta(engine);

        double current_time = 0.0;
        int counter = 0; // Compte les itérations avant un enregistrement
        double D_est = 0.0;

        for (int step = 0; step < N_tot; ++step) {
            current_time += h;
            E.getDomain().MakeLoop(*this);

            bool tumble_outcome = tumble(engine);
            collision = this->CollisionDetection(E);

            if (collision == nullptr) {
                if (tumble_outcome) {
                    if (Reorientation_mode == "run_and_reverse") {
                        double Xtmp = 2 * std::asin(2 * U(engine) - 1);
                        this->Theta = this->Theta + M_PI / 2 + Xtmp;
                    }
                    else if (Reorientation_mode == "isotropic") {
                        this->Theta = theta(engine);
                    }
                }
                Coord = Coord + Point(v0 * h * std::cos(Theta), v0 * h * std::sin(Theta));
            }
            else {
                collision->edgeProjection(*this);
                bool escape_outcome = escape(engine);
                if (tumble_outcome && escape_outcome) {
                    std::uniform_real_distribution<double> phi(
                        collision->escapeAngle(*this)[0],
                        collision->escapeAngle(*this)[1]);
                    double phi_outcome = phi(engine);
                    this->Theta = phi_outcome;
                    Coord = Coord + Point(v0 * h * std::cos(Theta), v0 * h * std::sin(Theta));
                }
            }

            // Enregistrement toutes les N_interval itérations
            if (++counter == N_interval) {
                counter = 0;
                double t_i = current_time;

                double dx = this->getFreeCoord().getx() - X.getx();
                double dy = this->getFreeCoord().gety() - X.gety();
                double Dist_2 = dx * dx + dy * dy;

                D_est = Dist_2 / (4.0 * t_i); // en 2D
                ofs << D_est;

                if (step < N_tot - 1) ofs << ",";
            }
        }

        ofs << "\n";
    }

    ofs.close();

    std::chrono::high_resolution_clock::time_point b = std::chrono::high_resolution_clock::now();
    unsigned int time = std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    std::cout << "temps d'execution de la mesure de D sur une periode de " << time_upper_bound
              << " avec " << N_samples << " echantillons et " << N_data
              << " mesures : " << time * 0.000001 << " s" << std::endl;
}


void Mobile::measure_displacement(Environment & E, double T, double h, Point X, std::string Reorientation_mode, const std::string & filename, int N_samples){
    //cette méthode sert à écrire dans un fichier csv un échantillon de N_samples coordonnées finales
    //après simulation : colonne 0 contient les abcisses et colonne 1 les ordonnées, sur N_samples lignes

    //Ceci me sert à vérifier numériquement la valeur théorique de n et de tau pour observer la convergence en loi des coordonnées
    //en simulation en espace libre vers une gaussienne de moyenne (0, 0) et de variance I2, convergence avec
    //taux d'erreur à moins de 5% me demande théoriquement n > 34792 pour une valeur d'erreur en probabilité < 5% avec 
    //tau = 0.02. (je dérive ceci de la généralisation multivariée du théorème de berry-esseen)
    std::random_device rd;
    uint64_t seed = 0;
    std::ofstream os;
    os.open(filename);
    int n = std::floor(T*(1/this->Tau)); // esperance du processus de comptage de tumble au temps final
    double C = std::sqrt(n)*(this->Tau)*(this->v0); //constante de renormalisation des coordonnées au temps final
    if (os.is_open() == false){
        std::cerr<<"measure_dispacement : impossible d'ouvrir le fichier"<<std::endl;
    }
    for (int i=0; i < N_samples ; i++) {
        seed = static_cast<uint64_t>(std::random_device{}());
        this->simulation(E, T, h, X, seed, Reorientation_mode);
        //on renormalise les coordonnées et on les écrit dans le fichier 
        os << (this->Coord).getx()/C << "," << (this->Coord).gety()/C <<std::endl;

    }
    os.close();
    return;
}


void Mobile::diffusivity_function_of_tau(Environment & E, double h, Point X, std::string Reorientation_mode,
                                        const std::string & filename, double tau_upper_bound, 
                                        int N_samples, int N_data, double time_upper_bound)
{
    std::chrono::high_resolution_clock::time_point a = std::chrono::high_resolution_clock::now();
    std::ofstream ofs;
    uint64_t seed = 0;
    ofs.open(filename);
    if (!ofs.is_open()) {
        std::cerr<<"diffusivity_function_of_tau : impossible d'ouvrir le fichier"<<std::endl;
        return;
    }
    /*
    on va stocker dans un fichier csv de N_samples lignes et de N_data colonnes les observations (samples) de D pour différentes
    valeurs de tau dans l'intervalle [0, tau_upper_bound] (il y en a donc N_data).
    */
    
    // on stocke les valeurs de tau en en-tete : 
    for (int i=0 ; i < N_data ; i++) {
        ofs << (i+1)*(tau_upper_bound / N_data) << std::endl;
    }

    for (int sample=0 ; sample < N_samples ; sample++)
    {
    //on écrit une ligne du csv correspondant à un échantillon

    //on écrit ligne par ligne les estimations de D pour différentes valeurs de tau : 
    for (int i_data=0 ; i_data < N_data - 1; i_data++) {
        this->Tau = (i_data + 1) * (tau_upper_bound / N_data);
        seed = static_cast<uint64_t>(std::random_device{}());
        simulation(E, time_upper_bound, h, X, seed, Reorientation_mode);
        ofs << (Mt * Mt) / (2 * 2 * time_upper_bound) << ","; // on écrit l'estimation de D
    }
    this->Tau = tau_upper_bound;
    seed = static_cast<uint64_t>(std::random_device{}());
    simulation(E, time_upper_bound, h, X, seed, Reorientation_mode);
    ofs << (Mt * Mt) / (2 * 2 * time_upper_bound) << ","; // on écrit l'estimation de D

    ofs <<"\n";
    }

    ofs.close();
    std::chrono::high_resolution_clock::time_point b = std::chrono::high_resolution_clock::now();
    unsigned int time = std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    std::cout << "temps d'execution de la mesure de D en fonction de tau pour tau variant de 0 a "<<tau_upper_bound
    <<", en estimant D a l'aide de "<<N_samples<<", et en effectuant "<<N_data<<" mesures : "<<time*.000001<<" s"<<std::endl;;
}