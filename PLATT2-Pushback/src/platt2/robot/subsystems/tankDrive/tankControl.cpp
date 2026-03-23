#include "platt2/robot/subsystems/tankDrive/tankControl.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include <cmath>
#include <math.h>
#include <memory>
#include <utility>
#include <vector>


namespace platt2{
namespace robot{
namespace subsystems{
namespace tankDrive{

void TankControl::moveToPoint(odometry::Position target, double maxV, double maxW, double timeout) {

    TankDrive::MovementVector motionVector;

    target.heading = target.heading*(M_PI/180);
    std::vector<point> path = generatePath(target);

    bool exitCondition = false;

    for (auto target:path){

        while (true){





        }
        if (exitCondition){break;}
    }
    motionVector.r = 0;
    motionVector.w = 0;
    drivetrain->moveVector(motionVector);
}

std::vector<TankControl::point> TankControl::generatePath(odometry::Position endPos) {
    
    std::vector<point> path;
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

    double n = length/0.1;

    for (int i = 0; i <= n; i++) {
        double t = (double)i / n;
        path.push_back({x(t), y(t)});
    }

    return path;

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

double TankControl::arcLength(odometry::Position p0, point p1, point p2, odometry::Position p3) {
    
    auto dx = [&](double t){
        return 3*pow(1-t,2)*(p1.x-p0.x)
             + 6*(1-t)*t*(p2.x-p1.x)
             + 3*pow(t,2)*(p3.x-p2.x);
        };

    auto dy = [&](double t){
        return 3*pow(1-t,2)*(p1.y-p0.y)
             + 6*(1-t)*t*(p2.y-p1.y)
             + 3*pow(t,2)*(p3.y-p2.y);
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
}


TankControl::TankControl(std::shared_ptr<TankDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID)
{
    drivetrain = drive;
    odometry = odom;
    this->positionPID = std::move(posPID);

}

}
}
}}
