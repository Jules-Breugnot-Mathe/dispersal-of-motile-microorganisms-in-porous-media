#include "Mobile.hpp"
#include "Environment.hpp"

# define M_PI           3.14159265358979323846  /* pi */

Mobile::Mobile(Point P, double v0, double Dr, double Tau, double Theta, double mu){
    this->v0 = v0; this->Dr = Dr; this->Tau = Tau; this->Theta = Theta; this->Coord = P; this->mu = mu;
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

const double Mobile::getMt() const{
    return this->Mt;
}

Point& Mobile::getCoord(){
    return this->Coord;
}

double Mobile::getV0ForDebug() const {
    return v0; 
}
double Mobile::getThetaForDebug() const {
    return Theta;
}


Solid* Mobile::CollisionDetection(Environment & E){ 
    //On parcours l'ensemble des corps de l'environnement à l'aide d'un itérateur, 
    //et on renvoie en sortie un pointeur de base sur le corps en collision,
    //ou le pointeur nul si aucune collision
    //std::cout<<"adresse de E : "<<&E<<std::endl;
    std::vector<std::unique_ptr<Solid>>::iterator it;
    
    for (it=E.get_Solid_vector().begin(); it != E.get_Solid_vector().end(); it++){
        if ((*it)->IsCollided(*this)){ // on déréférence l'itérateur pour avoir un pointeur sur Solid
            return (*it).get(); // on renvoie un pointeur de base sur Solid
        }
    }
    return nullptr;  // en absence de collision on renvoie un pointeur nul
}



void Mobile::simulation(Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode){
    int N = std::floor(T/h);
    this->Coord = X;
    std::mt19937_64 engine(seed);
    std::bernoulli_distribution tumble(h/this->Tau); // variable de bernoulli de l'evènement "tumbling"
    std::bernoulli_distribution escape(this->mu); // variable de bernoulli de l'évènement "escape"
    std::uniform_real_distribution<double> theta(0, 2*M_PI); // variable d'angle de réorientation 
    std::uniform_real_distribution<double> U(0, 1);
    bool tumble_outcome; // observation de la variable tumbling
    bool escape_outcome; // observation de la variable escape

    Solid* collision = nullptr; // on initialise la collision
    this->Theta = theta(engine); // on initialise l'orientation aléatoire

    for (int j=0;j<N;j++){ //on parcours le temps de 0 à T = Nh
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
            std::cout<<"collision avec le disque "<<collision->getid()<<" au temps "<<j*h<<std::endl;
            // collision détectée 
            // il faut projeter la particule sur la frontière du solide pour ne pas créer d'erreur avec le calcul d'angle 
            std::cout<<"coordonnees avant projection : "<<this->Coord<<std::endl;
            collision->edgeProjection(*this);
            std::cout<<"coordonnees apres projection : "<<this->Coord<<std::endl;
            escape_outcome = escape(engine); // on observe la variable escape (indépendance?)
           if ((tumble_outcome == 1)&&(escape_outcome == 1)) {
                std::cout<<"on s echappe du disque "<<collision->getid()<<std::endl;
                // le mobile s'échappe : on calcule l'intervalle dans lequel simuler une variable d'angle uniforme (angle de sortie) 
                std::uniform_real_distribution<double> phi(collision->escapeAngle(*this)[0], collision->escapeAngle(*this)[1]); 
                // variable d'angle de réorientation. Collision pointe sur un Solid, on appelle la méthode virtuelle pure escapeAngle,
                //qui se spécifie selon le solide, puis on en tire les bornes inf et sup pour l'angle.
                std::cout<<"intervalle d'angle de reorientation : ["<<collision->escapeAngle(*this)[0]<<" , "<<collision->escapeAngle(*this)[1]<<"]"<<std::endl;
                this->Theta = phi(engine); // on se réoriente
                Coord = Coord + Point(v0*h*std::cos(Theta), v0*h*std::sin(Theta)); // et on avance
            }
            else {
                // le mobile ne bouge pas
                std::cout<<"on reste coince sur le disque "<<collision->getid()<<std::endl;
            }
        }
    }
    this->Mt = std::sqrt((this->getCoord().getx() - X.getx())*(this->getCoord().getx() - X.getx()) + (this->getCoord().gety() - X.gety())*(this->getCoord().gety() - X.gety()));
    // On a calculé le déplacement total observé Mt
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


void Mobile::write_trajectory(Environment & E, double T, double h, Point X, uint64_t seed, std::string Reorientation_mode, const std::string & filename){
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

        ofs <<this->Coord.getx()<<" , "<<this->Coord.gety()<<std::endl; // on écrit les coordonnées dans le fichier

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
    this->Mt = std::sqrt((this->getCoord().getx() - X.getx())*(this->getCoord().getx() - X.getx()) + (this->getCoord().gety() - X.gety())*(this->getCoord().gety() - X.gety()));
    // On a calculé le déplacement total observé Mt
    ofs.close();
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
    obtenu par l'équivalent multivarié du théorème de berry-esseen, dans le cas particulier du déplacement libre dans R2
    */

    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "measure_diffusivity : impossible d'ouvrir le fichier\n";
        return;
    }

    double T = time_upper_bound / N_data; // durée d'un sous-segment
    if (T <= 0.0) {
        std::cerr << "measure_diffusivity : T invalide\n";
        return;
    }

    // Optionnel: écrire la première ligne contenant les temps (T, 2T, ..., N_data*T)
    for (int i = 0; i < N_data; ++i) {
        double t_i = T * (i + 1);
        ofs << t_i;
        if (i < N_data - 1) ofs << ",";
    }
    ofs << "\n";

    double Dist = 0; // on va stocker la distance totale parcourue pour éviter de passer par this->Mt 

    for (int j = 0; j < N_samples; ++j) {
        // Reset de la position pour la trajectoire j
        this->Coord = X;

        for (int i = 0; i < N_data; ++i) {
            // starting_point = position actuelle (cumulative)
            Point starting_point = this->Coord;

            // simulate one segment of duration T, starting from starting_point
            uint64_t seed = static_cast<uint64_t>(std::random_device{}());
            this->simulation(E, T, h, starting_point, seed, Reorientation_mode);

            // Après simulation, this->Coord est la position cumulée à temps (i+1)*T.
            Dist = std::sqrt(((this->getCoord().getx())*(this->getCoord().getx())) + (this->getCoord().gety())*(this->getCoord().gety()));
            double t_i = T * (i + 1);

            double D_est = std::numeric_limits<double>::quiet_NaN();
            if (t_i > 0.0 && std::isfinite(Mt)) {
                D_est = (Dist * Dist) / (4.0 * t_i); // normalisation en 2D
            } else {
                // on met NaN pour indiquer problème (sera ignoré par np.nanmean)
                D_est = std::numeric_limits<double>::quiet_NaN();
            }

            ofs << D_est;
            if (i < N_data - 1) ofs << ",";
        } // fin boucle i

        ofs << "\n";
    } // fin boucle j

    ofs.close();
}


void Mobile::diffusivity_function_of_tau(Environment & E, double h, Point X, std::string Reorientation_mode, 
                                         const std::string & filename, double time_upper_bound) {
    std::ofstream ofs;
    ofs.open(filename);
    if (ofs.is_open() == false) {
        std::cerr << "diffusivity_function_of_tau : impossible d'ouvrir le fichier" << std::endl;
        return;
    }

    int N_tau = 100; // nombre d'échantillons de tau
    int N_samples = 100; // nombre de simulations par tau pour estimer D
    double tau_min = 0.1; // borne inférieure de tau
    double tau_max = 10;  // borne supérieure de tau
    double D_tau = 0; // estimateur de D pour chaque tau
    double tau_value = 0; // valeur actuelle de tau
    uint64_t seed = 0;

    ofs << "tau,D" << std::endl; // en-tête du fichier CSV

    for (int j = 0; j < N_tau; j++) {
        tau_value = tau_min + j * (tau_max - tau_min) / (N_tau - 1);
        this->Tau = 1/tau_value; // mise à jour du paramètre tau du mobile
        D_tau = 0;

        for (int i = 0; i < N_samples; i++) {
            seed = static_cast<uint64_t>(std::random_device{}());
            this->simulation(E, time_upper_bound, h, X, seed, Reorientation_mode);
            // D = Mt / (2 * d * t)
            double D_i = this->Mt / (2.0 * 2.0 * time_upper_bound);
            D_tau += D_i;
        }

        D_tau = D_tau / N_samples; // moyenne empirique sur les échantillons

        ofs << std::fixed << std::setprecision(6) << tau_value << "," << D_tau << std::endl;
    }

    ofs.close();
    return;
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
