#ifndef DOMAIN_HPP
#define DOMAIN_HPP

class Mobile;

class Domain {
    private : 
        double radius;
    public : 
        Domain(double r = 1);
        ~Domain();
        Domain(const Domain & other);
        Domain& operator=(const Domain & other);
        const double& getradius() const;
        void MakeLoop(Mobile & M) const ;
};


#endif //DOMAIN_HPP