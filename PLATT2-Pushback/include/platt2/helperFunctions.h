#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <cmath>
#include <numeric>
#include <vector>
struct cart {

    double x;
    double y;

};

struct polar {

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

inline polar CtoP(double x, double y)  {

    polar p;

    p.r = sqrtf(pow(x, 2)+pow(y, 2));
    p.theta = atan2f(y, x);

    return p;

}


inline double rollAverage(double var, std::vector<double> data){

    data.emplace_back(var);
    data.erase(data.begin());

    return (std::accumulate(data.begin(),data.end(), (double)0)/data.size());

}

#endif