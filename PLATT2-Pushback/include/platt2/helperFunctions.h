#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <cmath>
#include <numeric>
#include <vector>
#include <array>


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

inline double distanceFromSecant(double x1, double y1,
                          double x2, double y2,
                          double x0, double y0)
{
    double vx = x2 - x1;
    double vy = y2 - y1;

    double wx = x0 - x1;
    double wy = y0 - y1;

    double cross = vx * wy - vy * wx;
    double dist = std::abs(cross) / std::sqrt(vx*vx + vy*vy);

    return dist;
}

inline double pythag(double P1[2], double P2[2]){

    double X = P2[0] - P1[0];
    double Y = P2[1] - P1[1];

    double hyp = std::sqrt(pow(X,2)+pow(Y,2));

    return hyp;

}


inline std::array<std::array<double,3>,3> inverse(std::array<std::array<double,3>,3> A){

    double det =
          A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
        - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
        + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    double invDet = 1.0 / det;

    std::array<std::array<double, 3>, 3> inv{};

    inv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invDet;
    inv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invDet;
    inv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invDet;

    inv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invDet;
    inv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invDet;
    inv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invDet;

    inv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invDet;
    inv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invDet;
    inv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invDet;

    return inv;

}


inline std::array<double, 3> matrixMult(std::array<std::array<double, 3>, 3> A, std::array<double, 3> B){
   
    std::array<double, 3> result{0.0, 0.0, 0.0};

    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t k = 0; k < 3; ++k)
        {
            result[i] += A[i][k] * B[k];
        }
    }

    return result;
}

inline std::array<std::array<double, 3>, 3> scale(std::array<std::array<double, 3>, 3> A, double s){

    std::array<std::array<double, 3>, 3> result{};

    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            result[i][j] = s * A[i][j];
        }
    }

    return result;

}

#endif