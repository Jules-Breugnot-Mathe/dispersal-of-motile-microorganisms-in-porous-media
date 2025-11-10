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
    std::cout<<"simulation d'une trajectoire sur un temps T = "<<T<<" : "<<time*0.000001 <<" s"<<std::endl; // temps affiché en secondes

    return;
}






void Mobile::simulation_expo(const Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode){
    double N = T/h;

    this->Mt = 0;
    this->Coord = X;
    this->Free_coord = X;
    this->Loop = {0, 0};
    
    std::mt19937_64 engine(seed);

    std::uniform_real_distribution<double> U(0, 1);
    std::geometric_distribution<int> Ti(((1/Tau)*T)/(N+1)); // on stocke des temps géométriques pour approximer le processus de poisson
    std::bernoulli_distribution escape(this->mu); // variable booléenne qui décide si on échappe à l'obstacle
    std::uniform_real_distribution<double> theta(0, 2*M_PI); // variable d'angle de réorientation 


    Solid* collision = nullptr; // on initialise la collision
    this->Theta = theta(engine); // on initialise l'angle de départ aléatoire
    // on a d'abord besoin de simuler les temps d'attente de tumbling
    std::list<std::pair<int, bool>> random_events;  // liste de temps d'attentes Ti suivant une loi exponentielle, avec escape booléen
    int T_outcome = 0; //temps exponentiels et versions discrètes
    double T_sum = 0;
    double* bounds = nullptr;
    //---------------------------INITIALISATION DES TEMPS----------------------//
    while (true) {
        T_outcome = Ti(engine);
        if (T_sum + T_outcome > N) break;   
        T_sum += T_outcome;
        random_events.push_front({T_outcome, escape(engine)}); 
    }

    random_events.push_front({0, 1}); // on insère un temps d'attente nul au début pour pouvoir décrémenter it sans problème
    random_events.push_back({T-T_sum, 1}); // on insère un dernier temps d'attente à la fin pour finir le processus

    std::list<std::pair<int, bool>>::iterator it = random_events.begin();
    ++it; // on fait pointer l'itérateur sur le deuxième élément de la liste random_events


    //------------------------DEBUT DE LA SIMULATION-------------------------//
    for (; it != random_events.end(); it++){ //on va parcourir les temps d'attente exponentiels

        collision = this->CollisionDetection(E); // update des collisions

        E.getDomain().MakeLoop(*this); // update d'atteinte de la bordure

        if ((collision == nullptr)||std::prev(it)->second){ // pas de collison ou dernier escape positif
            
            for (int j=0 ; j < it->first ; j++){ // on va avancer du nombre de pas stocké dans random_events
                Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // on avance
                Free_coord = Free_coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta));

                collision = this->CollisionDetection(E);
                E.getDomain().MakeLoop(*this);

                if (collision != nullptr) { // collision detectée 
                    
                    collision->edgeProjection(*this); // on projette le mobile sur la frontière de l'obstacle
                    bounds = collision->escapeAngle(*this);
                    std::uniform_real_distribution<double> alpha(bounds[0], bounds[1]);
                    this->Theta = alpha(engine);
                    break; // on revient à l'itérateur sur random_events
                }
                // en l'absence de collision, on se réoriente de facon aléatoire 
            }
            if (Reorientation_mode == "isotropic") {
                //on choisit un angle dans l'intervalle [0, 2*pi]
                this->Theta = theta(engine);
            }
            else if (Reorientation_mode == "run_and_reverse"){
                //on choisit un angle centré sur l'angle opposé à theta actuel
                double X = 2*std::asin(2*U(engine) - 1);
                this->Theta = this->Theta + M_PI/2 + X;
            }
        }
    }
    delete[] bounds;
    this->Mt = std::sqrt((this->getFreeCoord().getx() - X.getx())*(this->getFreeCoord().getx() - X.getx()) + (this->getFreeCoord().gety() - X.gety())*(this->getFreeCoord().gety() - X.gety()));
    // On a calculé le déplacement total observé Mt

}
 

void Mobile::write_trajectory_expo(const Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename)
{
    std::chrono::high_resolution_clock::time_point a= std::chrono::high_resolution_clock::now();
    std::ofstream ofs;
    ofs.open(filename, std::ofstream::out | std::ofstream::trunc);
    if (ofs.is_open() == false) {
        std::cerr<<"Mobile::write_trajectory_expo : impossible d'ouvrir le fichier"<<std::endl;
        return ;
    }

    /* CORPS DE LA FONCTION SIMULATION_EXPO */

    double N = T/h;
    this->Coord = X;

    std::mt19937_64 engine(seed);

    std::uniform_real_distribution<double> U(0, 1);
    std::geometric_distribution<int> Ti(((1/Tau)*T)/(N+1)); // on stocke des temps géométriques pour approximer le processus de poisson
    std::bernoulli_distribution escape(this->mu); // variable booléenne qui décide si on échappe à l'obstacle
    std::uniform_real_distribution<double> theta(0, 2*M_PI); // variable d'angle de réorientation 


    Solid* collision = nullptr; // on initialise la collision
    this->Theta = theta(engine); // on initialise l'angle de départ aléatoire
    // on a d'abord besoin de simuler les temps d'attente de tumbling
    std::list<std::pair<int, bool>> random_events;  // liste de temps d'attentes Ti suivant une loi exponentielle, avec escape booléen
    int T_outcome = 0; //temps exponentiels et versions discrètes
    double T_sum = 0;
    double* bounds = nullptr;
    //---------------------------INITIALISATION DES TEMPS----------------------//
    while (true) {
        T_outcome = Ti(engine);
        if (T_sum + T_outcome > N) break;   
        T_sum += T_outcome;
        random_events.push_front({T_outcome, escape(engine)}); 
    }

    random_events.push_front({0, 1}); // on insère un temps d'attente nul au début pour pouvoir décrémenter it sans problème
    random_events.push_back({T-T_sum, 1}); // on insère un dernier temps d'attente à la fin pour finir le processus

    std::list<std::pair<int, bool>>::iterator it = random_events.begin();
    ++it; // on fait pointer l'itérateur sur le deuxième élément de la liste random_events


    //------------------------DEBUT DE LA SIMULATION-------------------------//
    for (; it != random_events.end(); it++){ //on va parcourir les temps d'attente exponentiels

        collision = this->CollisionDetection(E); // update des collisions

        E.getDomain().MakeLoop(*this); // update d'atteinte de la bordure

        if ((collision == nullptr)||std::prev(it)->second){ // pas de collison ou dernier escape positif

            ofs <<this->Free_coord.getx()<<","<<this->Free_coord.gety()<<std::endl; // on écrit les coordonnées dans le fichier

            
            for (int j=0 ; j < it->first ; j++){ // on va avancer du nombre de pas stocké dans random_events
                Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // on avance
                Free_coord = Free_coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta));

                collision = this->CollisionDetection(E);
                E.getDomain().MakeLoop(*this);

                if (collision != nullptr) { // collision detectée 
                    
                    collision->edgeProjection(*this); // on projette le mobile sur la frontière de l'obstacle
                    bounds = collision->escapeAngle(*this);
                    std::uniform_real_distribution<double> alpha(bounds[0], bounds[1]);
                    this->Theta = alpha(engine);
                    break; // on revient à l'itérateur sur random_events
                }
                // en l'absence de collision, on se réoriente de facon aléatoire 
            }
            if (collision == nullptr) {

            if (Reorientation_mode == "isotropic") {
                //on choisit un angle dans l'intervalle [0, 2*pi]
                this->Theta = theta(engine);
            }
            else if (Reorientation_mode == "run_and_reverse"){
                //on choisit un angle centré sur l'angle opposé à theta actuel
                double X = 2*std::asin(2*U(engine) - 1);
                this->Theta = this->Theta + M_PI/2 + X;
            }
            }

        }
    }
    delete[] bounds;
    this->Mt = std::sqrt((this->getFreeCoord().getx() - X.getx())*(this->getFreeCoord().getx() - X.getx()) + (this->getFreeCoord().gety() - X.gety())*(this->getFreeCoord().gety() - X.gety()));
    // On a calculé le déplacement total observé Mt




    /* FIN DU CORPS DE LA FONCTION SIMULATION_EXPO */


    ofs.close();
    // temps courant, apres l'execution
    std::chrono::high_resolution_clock::time_point b= std::chrono::high_resolution_clock::now();
    // mesurer la difference, et l'exprimer en microsecondes 
    unsigned int time= std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    std::cout<<"Simulation du processus de Poisson par temps geometriques, "<<std::endl<<"ecriture d'une trajectoire sur un temps T = "<<T<<" : "<<time*0.000001 <<" s"<<std::endl; // temps affiché en secondes

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
    std::cout<<"Simulation du processus par somme de Bernoulli, "<<std::endl<<"ecriture d'une trajectoire sur un temps T = "<<T<<" : "<<time*0.000001 <<" s"<<std::endl; // temps affiché en secondes

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


void Mobile::measure_diffusivity(Environment & E, Point X,
                                 std::string Reorientation_mode,
                                 const std::string & filename,
                                 int N_samples, int N_data)
{
    /*
    on va stocker dans un fichier csv de N_samples lignes et de N_data colonnes les observations (samples) de D à différents
    temps de simulation (il y en a donc N_data). D est définis comme la valeur Mt^2 renormalisée par 2dt. Le but est d'observer numériquement
    à partir de quel temps de simulation D converge à moins de 5% d'erreur, comparé à la valeur théorique
    obtenu par l'équivalent multivarié du théorème de berry-esseen, dans le cas particulier du déplacement libre dans R2.
    On cherche également à savoir si le temps de convergence de D dans le cas non trivial est le même, et à mesurer D en différents milieux
    */

    double h = std::min(Tau/200, 0.01);

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

    double T = (this->Tau)*500;       // durée totale de la simulation,  
    // le temps de convergence de la simulation dépend linéairement de tau
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
    std::cout << "temps d'execution de la mesure de D "
              << " avec " << N_samples << " echantillons et " << N_data
              << " mesures : " << time * 0.000001 << " s" << std::endl;
}


double Mobile::measure_diffusivity_expo(const Environment & E, Point X,
                             std::string Reorientation_mode,
                              int N_samples)
{

    double h = std::min(Tau/200, 0.01);
    

    std::chrono::high_resolution_clock::time_point a = std::chrono::high_resolution_clock::now();

    if ((std::abs(X.getx()) >= E.getDomain().getradius()) || (std::abs(X.gety()) >= E.getDomain().getradius())) {
        std::cerr << "measure_diffusivity_expo : le point de départ des simulations doit être dans le domaine de l'environnement" << std::endl;
        return -1;
    }

    std::random_device rd;
    double D = 0;
    uint64_t seed = 0;
    double time_of_simu = Tau*5000;

    for (int i=0 ; i<N_samples ; i++) {
        seed = static_cast<uint64_t>(std::random_device{}());
        simulation_expo(E, time_of_simu, h, X, seed, Reorientation_mode);
        D += (Mt * Mt) / (2 * 2 * time_of_simu);
    }
    D /= N_samples;
    std::cout<<"Mesure effectuee : D = "<<D<<std::endl;
    


    std::chrono::high_resolution_clock::time_point b = std::chrono::high_resolution_clock::now();
    unsigned int time = std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    std::cout << "temps d'execution de la mesure de D "
              << " avec " << N_samples << " echantillons : "
               << time * 0.000001 << " s" << std::endl;
    return D;
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


void Mobile::diffusivity_function_of_tau(const Environment & E, Point X, std::string Reorientation_mode,
                                        const std::string & filename, double tau_upper_bound,
                                        int N_samples, int N_data)
{

    std::chrono::high_resolution_clock::time_point a = std::chrono::high_resolution_clock::now();
    std::ofstream ofs;
    uint64_t seed = 0;

    double h = std::min(Tau/200, 0.01);

    ofs.open(filename);
    if (!ofs.is_open()) {
        std::cerr<<"diffusivity_function_of_tau : impossible d'ouvrir le fichier"<<std::endl;
        return;
    }

    std::array<double, 2> approx_results = max_tau_bissection_approx(E, X, Reorientation_mode, tau_upper_bound, N_samples, 0.01);
    double tau_star = approx_results.data()[0];

    /*
    On va calculer une estimation de D* et tau* dans cette configuration. Ensuite, on veux faire varier tau entre 
    0 et 100 fois tau*. On calcule ensuite N_data valeurs de D correspondantes dans cet intervalle, puis on les affichera 
    en faisant varier tau en échelle logarithmique
    */

    double tau_var = 0;
    double D_estimate = 0;
    double log_ratio = 0;
    
    // on stocke les valeurs de tau en en-tete : 
    for (int i = 0; i < N_data; i++) {
    log_ratio = -2.0 + 4.0 * static_cast<double>(i) / (N_data - 1);
    tau_var = tau_star * std::pow(10.0, log_ratio);
    ofs << tau_var ;
    if (i < N_data - 1) ofs << ",";
    }
    ofs << "\n";
    
    //on va écrire la deuxième ligne du csv contenant les valeurs de D
    for (int data = 0 ; data < N_data ; data++){
        D_estimate = 0;
        log_ratio = -2.0 + 4.0 * static_cast<double>(i) / (N_data - 1);
        tau_var = tau_star * std::pow(10.0, log_ratio);
        Tau = tau_var; 
        std::cout<<"tau = "<<tau_var<<std::endl;
        D_estimate = measure_diffusivity_expo(E, X, Reorientation_mode, N_samples);
        ofs << D_estimate;
        if (data < N_data - 1) {
            ofs << ",";
        }
    }


    ofs.close();
    std::chrono::high_resolution_clock::time_point b = std::chrono::high_resolution_clock::now();
    unsigned int time = std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    std::cout << "temps d'execution de la mesure de D en fonction de tau pour tau variant de 0 a "<<approx_results.data()[0]
    <<", en estimant D a l'aide de "<<N_samples<<", et en effectuant "<<N_data<<" mesures : "<<time*.000001<<" s"<<std::endl;
}


std::array<double, 2> Mobile::max_tau_bissection_approx(
    const Environment & E, Point X, std::string Reorientation_mode,
    double tau_upper_bound, int N_samples, double tol)
{
    /*
    estimation avec erreur de tol de la valeur de D* et tau* pour un ensemble de paramètre donné 
    par une méthode de bissection
    */
    auto start = std::chrono::high_resolution_clock::now();

    double h = std::min(Tau/200, 0.01);
    double time_upper_bound = 0;

    auto estimate_D = [&](double tau) -> double {
        double D_sum = 0.0;
        for (int i = 0; i < N_samples; i++) {
            this->Tau = tau;
            time_upper_bound = Tau*2000;
            this->Mt = 0.0; // reset le déplacement
            uint64_t seed = static_cast<uint64_t>(std::random_device{}());
            simulation_expo(E, time_upper_bound, h, X, seed, Reorientation_mode);
            D_sum += (Mt * Mt) / (4.0 * time_upper_bound);
        }
        return D_sum / N_samples;
    };

    double tau_left = 0.0;
    double tau_right = tau_upper_bound;
    double tau_mid1, tau_mid2;
    double D1, D2;

    // Paramètre pour le ratio doré (plus efficace qu'une bissection stricte)
    const double phi = (std::sqrt(5.0) - 1.0) / 2.0;

    int iteration = 0;
    while ((tau_right - tau_left) > tol) {
        iteration++;

        tau_mid1 = tau_right - phi * (tau_right - tau_left);
        tau_mid2 = tau_left + phi * (tau_right - tau_left);

        D1 = estimate_D(tau_mid1);
        D2 = estimate_D(tau_mid2);

        std::cout << "[Iteration " << iteration << "] "
                  << "tau1 = " << tau_mid1 << ", D1 = " << D1
                  << " | tau2 = " << tau_mid2 << ", D2 = " << D2 << std::endl;

        if (D1 < D2)
            tau_left = tau_mid1; // le maximum est à droite
        else
            tau_right = tau_mid2; // le maximum est à gauche
    }

    double tau_star = 0.5 * (tau_left + tau_right);
    double D_star = estimate_D(tau_star);

    std::chrono::duration<double> elapsed =
        std::chrono::high_resolution_clock::now() - start;

    std::cout << "\n========== Resultat ==========\n";
    std::cout << "Tau_star : " << tau_star << std::endl;
    std::cout << "D_star : " << D_star << std::endl;
    std::cout << "iterations : " << iteration << std::endl;
    std::cout << "Duree totale : " << elapsed.count() << " s\n";
    std::cout << "==============================\n";
    std::array<double, 2> results;
    results.data()[0] = tau_star; results.data()[1] = D_star;
    return results;
}
