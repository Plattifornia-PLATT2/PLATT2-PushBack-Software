#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <cmath>
struct cart {

    double x;
    double y;

};

struct polor {

    double theta;
    double r;

};

inline int sgn(double val) {
    return ((0) < val) - (val < (0));
}

inline cart PtoC(double r, double theta){
    
    cart p;
    
    p.x = r*cos(theta);
    p.y = r*sin(theta);

    return p;

}

inline polor CtoP(double x, double y)  {

    polor p;

    p.r = sqrtf(pow(x, 2)+pow(y, 2));
    p.theta = atan2f(y, x);

    return p;

}

#endif