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
    target.heading = target.heading+(M_PI);

    if(target.heading > M_PI || target.heading < -M_PI){
        target.heading = -1 * sgn(target.heading) * (2*M_PI - std::abs(target.heading));
    }  
    std::vector<odometry::Position> path = generatePath(target);
    
    std::vector<double> timeoutArray(50,1);
    std::vector<double> posArray(50,1);
    std::vector<double> angleArray(50,10);
    bool exitCondition = false;

    const double b = .6; //todo: tuned
    const double L = 0.2; //todo: tuned

    const double lookAheadDistance = 2;
    double currentVel = 0;
    bool usePID = false;
    int index = 0;

    positionPID->resetPID();
    headingPID->resetPID();

    for (auto pathTarget:path){

        while (true){
            
            odometry::Position currentPos = odometry->getPos();
            double distanceToTarget = distanceBetweenPoints({currentPos.x, currentPos.y}, {pathTarget.x, pathTarget.y});

            if ((distanceToTarget < lookAheadDistance) and index != path.size()-1){ 
                break; // Move to the next target in the path
            }

            double errorx = pathTarget.x - currentPos.x;
            double errory = pathTarget.y - currentPos.y;
            double eth = -pathTarget.heading + currentPos.heading;

            //adjust angle error for wraparound
            if(eth > M_PI || eth < -M_PI){
                eth = -1 * sgn(eth) * (2*M_PI - std::abs(eth));
            }  
    
            //convert to local robot coordinates    
            double ex = errorx*sin(currentPos.heading) - errory*cos(currentPos.heading);
            double ey = errorx*sin(currentPos.heading) + errory*cos(currentPos.heading);
            //std::cout <<eth<< ", " << ey << std::endl;

            double vd = trapezoidalVelocity(pathLength-remainingPathDistance(path, currentPos), pathLength, maxV, 1);
            double wd = eth*0.1;//headingPID//->calculate(0, eth);

            double k = 2*L*sqrt(pow(wd,2)+(b*pow(vd,2)));

            if (index == path.size()-1){

                std::cout << "Final Point Reached" << std::endl;
                bool posExit = false;
                bool angleExit = false;
                
                motionVector.v = vd;
                motionVector.w = headingPID->calculate(0, eth);

                avg angleAvg = rollAverage(eth, angleArray);
                angleArray = angleAvg.data;
                
                if (angleAvg.average < 0.025) {
                   bool angleExit = true;
                }
                
                avg posAvg = rollAverage(std::abs(sin(atan2(ey, ex))), posArray);
                posArray = posAvg.data;
                if (posAvg.average < 0.1) {
                    bool posExit = true; 
                }
                if (posExit && angleExit){
                    exitCondition = true;
                    break;
                }


            }else {
                
                motionVector.v = vd*cos(eth) + k*ey;
                motionVector.w = wd + k*eth + (eth != 0 ? ((b*vd*ex*sin(eth))/eth) : 1.0);

                
            }
            
            //motionVector.v = currentVel;
            //motionVector.w = ex;

            
            drivetrain->moveVector(motionVector);
            
            pros::delay(10);
        }
        index++;
        if (exitCondition){break;}
    }

    
            //avg timeoutAvg = rollAverage(odometry->getVelocity(), timeoutArray);
            //timeoutArray = timeoutAvg.data;
            
           // if (timeoutAvg.average < 0.01) {
           //     exitCondition = true;
           //     break; // Exit if the robot is stuck
           // }


    motionVector.v = 0;
    motionVector.w = 0;
    drivetrain->moveVector(motionVector);
}

std::vector<odometry::Position> TankControl::generatePath(odometry::Position endPos) {
    
    std::vector<odometry::Position> path;
    definedPath pathDef;
    pathDef.p0 = odometry->getPos();
    pathDef.p3 = endPos;

    double angleDiff = std::min(std::abs(endPos.heading - pathDef.p0.heading), 2 * M_PI - std::abs(endPos.heading - pathDef.p0.heading));
    double scaler = 4; // Adjus t this value to change the distance of control points from start and end positions

    
    pathDef.p1.x = pathDef.p0.x + angleDiff*scaler*cos(pathDef.p0.heading);
    pathDef.p1.y = pathDef.p0.y + angleDiff*scaler*sin(pathDef.p0.heading);
    pathDef.p2.x = pathDef.p3.x + angleDiff*scaler*cos(pathDef.p3.heading);
    pathDef.p2.y = pathDef.p3.y + angleDiff*scaler*sin(pathDef.p3.heading);

    pathLength = arcLength(pathDef);

    auto x = [&](double t){ 
        return pow((1-t),3)*pathDef.p0.x + 3*pow((1-t),2)*t*pathDef.p1.x + 3*(1-t)*pow(t,2)*pathDef.p2.x + pow(t,3)*pathDef.p3.x;
    };

    auto y = [&](double t){
        return pow((1-t),3)*pathDef.p0.y + 3*pow((1-t),2)*t*pathDef.p1.y + 3*(1-t)*pow(t,2)*pathDef.p2.y + pow(t,3)*pathDef.p3.y;
    };

    auto dx = [&](double t){
        return 3*pow(1-t,2)*(pathDef.p1.x-pathDef.p0.x) + 6*(1-t)*t*(pathDef.p2.x-pathDef.p1.x) + 3*pow(t,2)*(pathDef.p3.x-pathDef.p2.x);
    };

    auto dy = [&](double t){
        return 3*pow(1-t,2)*(pathDef.p1.y-pathDef.p0.y) + 6*(1-t)*t*(pathDef.p2.y-pathDef.p1.y) + 3*pow(t,2)*(pathDef.p3.y-pathDef.p2.y);
    };

    auto tangentAngle = [&](double t){
        return atan2(dy(t), dx(t));
    };

    double n = pathLength/0.5;

    for (int i = 0; i <= n; i++) {
        double t = (double)i / n;
        path.push_back({x(t), y(t), tangentAngle(t)});
    }

    return path;

}

double TankControl::arcLength(definedPath path) {
    
    auto dx = [&](double t){
        return 3*pow(1-t,2)*(path.p1.x-path.p0.x) + 6*(1-t)*t*(path.p2.x-path.p1.x) + 3*pow(t,2)*(path.p3.x-path.p2.x);
        };

    auto dy = [&](double t){
        return 3*pow(1-t,2)*(path.p1.y-path.p0.y) + 6*(1-t)*t*(path.p2.y-path.p1.y) + 3*pow(t,2)*(path.p3.y-path.p2.y);
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
        double max_accel_delta = maxAccel * 0.02;
        double output = std::clamp(currentVel + max_accel_delta, 0.0, maxVel);
        return output;

    } else {
        // --- Handoff to decel/approach controller ---
        usePID = true;
        return std::clamp((positionPID->calculate(remainingDistance, 0)), 0.0, maxVel);
    }
}

double TankControl::trapezoidalVelocity(double distanceTravelled, double totalDistance, double maxVel, double maxAccel) {
    
    // Distance needed to ramp up / down
    double rampDist = (maxVel * maxVel) / (2.0 * maxAccel);

    // If total distance is too short to reach maxVel, triangle profile
    double peakVel = maxVel;
    if (2.0 * rampDist > totalDistance) {
        peakVel = sqrt(maxAccel * totalDistance);
        rampDist = totalDistance / 2.0;
    }

    double remaining = totalDistance - distanceTravelled;

    double velFromAccel  = sqrt(2.0 * maxAccel * distanceTravelled); // ramp up
    double velFromDecel  = sqrt(2.0 * maxAccel * remaining);          // ramp down

    return std::clamp(std::min({velFromAccel, velFromDecel, peakVel}), 0.0, maxVel);
}
double TankControl::remainingPathDistance(std::vector<odometry::Position> path, odometry::Position currentPos) {
    
    // Find the closest point on the path to the robot's current position
    int closestIndex = 0;
    double closestDist = std::numeric_limits<double>::max();

    for (int i = 0; i < path.size(); i++) {
        double d = distanceBetweenPoints({currentPos.x, currentPos.y}, {path[i].x, path[i].y});
        if (d < closestDist) {
            closestDist = d;
            closestIndex = i;
        }
    }

    // Sum arc length from closest point to end of path
    double remaining = 0;
    for (int i = closestIndex; i < (int)path.size() - 1; i++) {
        remaining += distanceBetweenPoints({path[i].x, path[i].y}, {path[i+1].x, path[i+1].y});
    }

    return remaining;
}

void TankControl::turnToHeading(double targetAngle, double maxAngularVel) {
    
    TankDrive::MovementVector motionVector;
    odometry::Position currentPos;
    std::vector<double> angleArray(50,10);

    headingPID->resetPID();

    while (true) {
        
        currentPos = odometry->getPos();
        double angleError = targetAngle - currentPos.heading;

        if(angleError > M_PI || angleError < -M_PI){
            angleError = -1 * sgn(angleError) * (2*M_PI - std::abs(angleError));
        }  

        avg posAvg = rollAverage(std::abs(angleError), angleArray);
        angleArray = posAvg.data;
        if (posAvg.average < 0.025) {
            break; // Exit if the robot is within 0.025 radians of the target angle
        }


        motionVector.w = std::clamp(headingPID->calculate(0, angleError), -maxAngularVel, maxAngularVel);
        motionVector.v = 0;
        drivetrain->moveVector(motionVector);
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
