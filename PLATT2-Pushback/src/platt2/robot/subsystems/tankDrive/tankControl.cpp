#include "platt2/robot/subsystems/tankDrive/tankControl.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include "platt2/helperFunctions.h"


namespace platt2{
namespace robot{
namespace subsystems{
namespace tankDrive{

void TankControl::moveToPoint(odometry::Position target, double maxV, double maxW) {

    TankDrive::MovementVector motionVector;

    target.heading = target.heading*(M_PI/180);
    std::vector<odometry::Position> path = generatePath(target);
    
    std::vector<double> timeoutArray(50,1);
    bool exitCondition = false;

    const double b = 1; //todo: tuned
    const double L = 1; //todo: tuned

    const double lookAheadDistance = 0.5;
    double currentVel = 0;
    bool usePID = false;
    int index = 0;

    positionPID.reset();
    headingPID.reset();

    for (auto pathTarget:path){

        while (true){
            
            odometry::Position currentPos = odometry->getPos();
            double distanceToTarget = distanceBetweenPoints({currentPos.x, currentPos.y}, {pathTarget.x, pathTarget.y});

            if (distanceToTarget < lookAheadDistance) {
                break; // Move to the next target in the path
            }
            
            avg timeoutAvg = rollAverage(odometry->getVelocity(), timeoutArray);
            timeoutArray = timeoutAvg.data;
            
            if (timeoutAvg.average < 0.01) {
                exitCondition = true;
                break; // Exit if the robot is stuck
            }

            double errorx = pathTarget.x - currentPos.x;
            double errory = pathTarget.y - currentPos.y;
            double eth = pathTarget.heading - currentPos.heading;

            // adjust angle error for wraparound
            if(eth > M_PI || eth < -M_PI){
                eth = -1 * sgn(eth) * (2*M_PI - std::abs(eth));
            }  
            
            //convert to local robot coordinates    
            double ex = errorx*cos(currentPos.heading) + errory*sin(currentPos.heading);
            double ey = -errorx*sin(currentPos.heading) + errory*cos(currentPos.heading);

            currentVel = velocityProfile(path.size(), path.size()-index, currentVel, maxV, maxV, usePID);
            double vd = currentVel;
            double wd = headingPID->calculate(0, eth);

            double k = 2*L*sqrt(pow(wd,2)+(b*pow(vd,2)));

            motionVector.v = vd*cos(eth) + k*ex;
            motionVector.w = wd + k*eth + (b*vd*ey)*(eth != 0 ? sin(eth)/eth : 1.0);

            drivetrain->moveVector(motionVector);
            
            pros::delay(10);
        }
        index++;
        if (exitCondition){break;}
    }
    motionVector.v = 0;
    motionVector.w = 0;
    drivetrain->moveVector(motionVector);
}

std::vector<odometry::Position> TankControl::generatePath(odometry::Position endPos) {
    
    std::vector<odometry::Position> path;
    odometry::Position startPos = odometry->getPos();

    point control1;
    point control2;

    double angleDiff = std::min(std::abs(endPos.heading - startPos.heading), 2 * M_PI - std::abs(endPos.heading - startPos.heading));
    double scaler = 3; // Adjust this value to change the distance of control points from start and end positions

    control1.x = startPos.x + angleDiff*scaler*cos(startPos.heading);
    control1.y = startPos.y + angleDiff*scaler*sin(startPos.heading);
    control2.x = endPos.x - angleDiff*scaler*cos(endPos.heading);
    control2.y = endPos.y - angleDiff*scaler*sin(endPos.heading);

    double length = arcLength(startPos, control1, control2, endPos);

    auto x = [&](double t){
        return pow((1-t),3)*startPos.x + 3*pow((1-t),2)*t*control1.x + 3*(1-t)*pow(t,2)*control2.x + pow(t,3)*endPos.x;
    };

    auto y = [&](double t){
        return pow((1-t),3)*startPos.y + 3*pow((1-t),2)*t*control1.y + 3*(1-t)*pow(t,2)*control2.y + pow(t,3)*endPos.y;
    };

    auto dx = [&](double t){
        return 3*pow(1-t,2)*(control1.x-startPos.x) + 6*(1-t)*t*(control2.x-control1.x) + 3*pow(t,2)*(endPos.x-control2.x);
    };

    auto dy = [&](double t){
        return 3*pow(1-t,2)*(control1.y-startPos.y) + 6*(1-t)*t*(control2.y-control1.y) + 3*pow(t,2)*(endPos.y-control2.y);
    };

    auto tangentAngle = [&](double t){
        return atan2(dy(t), dx(t));
    };

    double n = length/0.1;

    for (int i = 0; i <= n; i++) {
        double t = (double)i / n;
        path.push_back({x(t), y(t), tangentAngle(t)});
    }

    return path;

}

double TankControl::arcLength(odometry::Position p0, point p1, point p2, odometry::Position p3) {
    
    auto dx = [&](double t){
        return 3*pow(1-t,2)*(p1.x-p0.x) + 6*(1-t)*t*(p2.x-p1.x) + 3*pow(t,2)*(p3.x-p2.x);
        };

    auto dy = [&](double t){
        return 3*pow(1-t,2)*(p1.y-p0.y) + 6*(1-t)*t*(p2.y-p1.y) + 3*pow(t,2)*(p3.y-p2.y);
    };

    auto vel = [&](double t){
        double vx = dx(t);
        double vy = dy(t);
        return sqrt(vx*vx + vy*vy);
    };

    int n = 100; 
    double length = 0;

    for(int i=0;i<n;i++)
    {
        double t0 = (double)i/n;
        double t1 = (double)(i+1)/n;
        double tm = (t0+t1)/2;

        length += (t1-t0)/6*(vel(t0) + 4*vel(tm) + vel(t1));
    }

    return length;
}

double TankControl::velocityProfile(double totalDistance, double remainingDistance, double currentVel, double maxVel, double maxAccel, bool& usePID) {

    double distanceTravelled = totalDistance - remainingDistance;
    bool reachedMaxVel       = currentVel >= maxVel;
    bool pastHalfway         = distanceTravelled >= totalDistance * 0.5;

    if ((!reachedMaxVel && !pastHalfway) && !usePID ) {
    //if (false) {
        // --- Acceleration phase ---
        double max_accel_delta = maxAccel * 0.01;
        double output = std::clamp(currentVel + max_accel_delta, 0.0, maxVel);
        return output;

    } else {
        // --- Handoff to decel/approach controller ---
        usePID = true;
        return std::clamp((positionPID->calculate(remainingDistance, 0)), 0.0, maxVel);
    }
}


TankControl::TankControl(std::shared_ptr<TankDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID)
{
    drivetrain = drive;
    odometry = odom;
    this->positionPID = std::move(posPID);
    this->headingPID = std::move(headingPID);

}

}
}
}}
