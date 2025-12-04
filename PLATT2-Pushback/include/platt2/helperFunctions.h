#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <cmath>
#include <numeric>
#include <vector>

/**
 * @brief A struct to represent a cartesian coordinate.
 * @author Logan Wolf
 */
struct cart {

    double x;
    double y;

};

/**
 * @brief A struct to represent a polar coordinate.
 * @author Logan Wolf
 */
struct polar {

    double theta;
    double r;

};

/**
 * @brief A struct to hold a rolling average.
 * @author Logan Wolf
 */
struct avg {
    double average;
    std::vector<double> data;
};

/**
 * @brief Returns the sign of a given value.
 * 
 * @param val The value to check.
 * @return int Returns 1 if positive, -1 if negative, and 0 if zero.
 */
inline int sgn(double val) {
    return ((0) < val) - (val < (0));
}

/**
 * @brief Converts polar coordinates to cartesian coordinates.
 * 
 * @param r The radius or distance from the origin.
 * @param theta The angle in radians.
 * @return cart The corresponding cartesian coordinates.
 */
inline cart PtoC(double r, double theta){
    
    cart p;
    
    p.x = r*cos(theta);
    p.y = r*sin(theta);

    return p;

}

/**
 * @brief Converts cartesian coordinates to polar coordinates.
 * 
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return polar The corresponding polar coordinates.
 */
inline polar CtoP(double x, double y)  {

    polar p;

    p.r = sqrtf(pow(x, 2)+pow(y, 2));
    p.theta = atan2f(y, x);

    return p;

}

/**
 * @brief Calculates a rolling average by adding a new value and removing the oldest one.
 * 
 * @param var The new value to add.
 * @param data The current data set.
 * @return avg The updated rolling average and data set.
 */
inline avg rollAverage(double var, std::vector<double> data){

    data.emplace_back(var);
    data.erase(data.begin());

    avg result;
    
    result.data = data;
    result.average = std::accumulate(data.begin(),data.end(), (double)0)/data.size();

    return result;

}

#endif