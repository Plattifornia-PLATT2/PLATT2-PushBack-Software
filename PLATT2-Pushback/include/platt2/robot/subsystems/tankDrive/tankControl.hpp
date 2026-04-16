#ifndef TANKCONTROL_HPP
#define TANKCONTROL_HPP

#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include "platt2/robot/subsystems/tankDrive/tankDrive.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/pid/pid.hpp"
#include "platt2/helperFunctions.h"

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

    /**
     * @brief Moves the holonomic drive to a specified point with a target heading.
     * 
     * @param x_target Target X value to move to.
     * @param y_target Target Y value to move to.
     * @param heading_target Target heading to achieve at the end of the movement.
     */
    void moveToPoint(odometry::Position target, double rSpeed = 0.8, double wSpeed = 0.4);
    void turnToHeading(double targetHeading, double maxAngularVel);

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

    struct definedPath{

        odometry::Position p0; 
        point p1; 
        point p2; 
        odometry::Position p3;
    
    };

    
    double velocityProfile(double TotalDistance, double remainingDistance, double currentVel, double maxVel, double maxAccel, bool& usePid);
    double trapezoidalVelocity(double distanceTravelled, double totalDistance, double maxVel, double maxAccel);
    double arcLength(definedPath path);
    std::vector<odometry::Position> generatePath(odometry::Position target);
    double remainingPathDistance(std::vector<odometry::Position> path, odometry::Position currentPos);

    double pathLength;


};
}}}}

#endif