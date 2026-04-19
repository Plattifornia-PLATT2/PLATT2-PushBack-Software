#ifndef TANKCONTROL_HPP
#define TANKCONTROL_HPP

#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include "platt2/robot/subsystems/tankDrive/tankDrive.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/pid/pid.hpp"
#include "platt2/helperFunctions.h"
#include "Eigen/Dense"
#include <memory>

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace subsystems{

/**
 * @brief Namespace for holonomic drive related code.
 * @authors PLATT2 Development team.
 */

/**
 * @brief Control class for holonomic drive systems.
 * @authors Dominic Young and Logan Wolf
 */
namespace tankDrive{

class TankControl{
public:
    /**
     * @brief Construct a new Holonomic Control object
     * 
     * @param drive X drive chassis to control
     * @param odom Odometry system for position tracking
     * @param posPID PID controller for position control
     * @param headingPID PID controller for heading control
     */
    TankControl(std::shared_ptr<TankDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID);
    
        struct parameter {
    double qx     = 2;   // longitudinal — keep low, feedforward handles this
    double qy     = 5.0;   // lateral — high, this is what LTV is best at correcting
    double qtheta = 15.0;   // heading — moderate
    double rV     = 1.0;   // resist large velocity corrections
    double rW     = 0.5; 
        };

    /**
     * @brief Moves the holonomic drive to a specified point with a target heading.
     * 
     * @param x_target Target X value to move to.
     * @param y_target Target Y value to move to.
     * @param heading_target Target heading to achieve at the end of the movement.
     */
    void moveToPoint(odometry::Position target, double rSpeed = 0.8, parameter params = {0,0,0,0,0});
    void turnToHeading(double targetHeading, double maxAngularVel=0.3);

    /**
     * @brief Used for debugging purposes.
     */

private:
    std::shared_ptr<TankDrive> drivetrain;
    std::shared_ptr<odometry::Odometry> odometry;
    std::unique_ptr<pid::PID> positionPID;
    std::unique_ptr<pid::PID> headingPID;

    struct point {
        double x;
        double y;
    };


    struct waypoint {
        odometry::Position pos;
        double v;
        double w;
    };



    odometry::Position p0; 
    point p1; 
    point p2; 
    odometry::Position p3;

    double tangentAngle(double t);
    double x(double t);
    double y(double t);
    double dx(double t);    
    double dy(double t);
    double ddx(double t);
    double ddy(double t);
    
    double curvature(double t);

    Eigen::Matrix3d solveRiccati(const Eigen::Matrix3d Ad,const Eigen::Matrix<double,3,2> Bd, const Eigen::Matrix3d Q, const Eigen::Matrix2d R, int iterations = 30);

    double trapezoidalVelocity(double t, double maxVel, double pathLength);
    double arcLength();
    std::vector<waypoint> generatePath(odometry::Position target);
    double remainingPathDistance(std::vector<waypoint> path, odometry::Position currentPos);
    void advanceIndex(std::vector<waypoint>& path, odometry::Position current);
    
    double pathLength;
    int waypointIndex;
    bool finished;

};
}}}}

#endif