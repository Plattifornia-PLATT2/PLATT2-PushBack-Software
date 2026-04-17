#include "platt2/robot/subsystems/tankDrive/tankControl.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include "platt2/helperFunctions.h"
#include "Eigen/Dense"

namespace platt2{
namespace robot{
namespace subsystems{
namespace tankDrive{

void TankControl::moveToPoint(odometry::Position target, double maxV, parameter params) {

    TankDrive::MovementVector motionVector;

    target.heading = target.heading*(M_PI/180);
    target.heading = wrapHeading(target.heading);
  
    std::vector<waypoint> path = generatePath(target);

    waypointIndex = 0;
    finished = false;

    Eigen::Matrix3d m_Q = Eigen::Matrix3d::Zero();
        m_Q(0, 0) = params.qx;
        m_Q(1, 1) = params.qy;
        m_Q(2, 2) = params.qtheta;
    
    Eigen::Matrix2d m_R = Eigen::Matrix2d::Zero();
        m_R(0, 0) = params.rV;
        m_R(1, 1) = params.rW;    
    
    while (!finished) {

        odometry::Position currentPos = odometry->getPos();

        advanceIndex(path, currentPos);

        isImpeded();

        const waypoint wp = path[waypointIndex];

        double cosRef = std::cos(wp.pos.heading);
        double sinRef = std::sin(wp.pos.heading);
 
        double dx = currentPos.x - wp.pos.x;
        double dy = currentPos.y - wp.pos.y;
 
        // Rotate global error into reference frame
        Eigen::Vector3d error;
        error(0) =  cosRef * dx + sinRef * dy;          
        error(1) = -sinRef * dx + cosRef * dy;       
        error(2) = wrapHeading(currentPos.heading - wp.pos.heading);

        double vRef = wp.v;

        if (std::abs(vRef) < 0.01) {
            motionVector.v = 0;
            motionVector.w = 0;
            drivetrain->moveVector(motionVector);
            pros::delay(10);
            continue;
        }
        
        Eigen::Matrix3d Ac = Eigen::Matrix3d::Zero();
        Ac(0, 1) =  wp.w;
        Ac(1, 0) = -wp.w;
        Ac(1, 2) =  vRef;

        Eigen::Matrix<double, 3, 2> Bc = Eigen::Matrix<double, 3, 2>::Zero();
        Bc(0, 0) = 1.0;
        Bc(2, 1) = 1.0;

        Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity() + Ac * 0.01;
        Eigen::Matrix<double, 3, 2> Bd = Bc * 0.01;

        Eigen::Matrix3d P = solveRiccati(Ad, Bd, m_Q, m_R);

        Eigen::Matrix2d S = m_R + Bd.transpose() * P * Bd;
        Eigen::Matrix<double, 2, 3> K = S.ldlt().solve(Bd.transpose() * P * Ad);

        Eigen::Vector2d correction = -K * error;
        
        if (std::isnan(correction(0)) || std::isnan(correction(1))) {
            correction << 0, 0;
        }

        motionVector.v = vRef + correction(0);
        motionVector.w = -(wp.w + correction(1));

        std::cout << motionVector.v << ", " << motionVector.w << std::endl;


        drivetrain->moveVector(motionVector);
        pros::delay(10);


    }

    std::cout << "Finished moving to point." << std::endl;
    
    motionVector.v = 0;
    motionVector.w = 0;

    drivetrain->moveVector(motionVector);
}

void TankControl::advanceIndex(std::vector<waypoint>& path, odometry::Position current) {
        constexpr double kSwitchRadius = 2;  // lookahead distance in inches 
        while (waypointIndex < static_cast<int>(path.size()) - 1) {
            const odometry::Position target = path[waypointIndex].pos;
            double dist = std::hypot(current.x - target.x,
                                     current.y - target.y);
            if (dist < kSwitchRadius) {
                ++waypointIndex;
            } else {
                //if (waypointIndex == static_cast<int>(path.size()) - 1) {
                //    finished = true;
                //}
                break;
            }
        }
    }

void TankControl::isImpeded() {

}




std::vector<TankControl::waypoint> TankControl::generatePath(odometry::Position endPos) {
    
    double tankWidth = 14.0; //TODO: add as parameter and tune
    double maxVel = 0.3; // TODO: add as parameter and tune

    std::vector<waypoint> path;
    p0 = odometry->getPos();
    p3 = endPos;

    double angleDiff = std::min(std::abs(endPos.heading+M_PI - p0.heading), 2 * M_PI - std::abs(endPos.heading+M_PI - p0.heading));
    double scaler = 4; //TODO: add as parameter and tune

    p1.x = p0.x + angleDiff*scaler*cos(p0.heading);
    p1.y = p0.y + angleDiff*scaler*sin(p0.heading);
    p2.x = p3.x + angleDiff*scaler*cos(p3.heading+M_PI);
    p2.y = p3.y + angleDiff*scaler*sin(p3.heading+M_PI);

    pathLength = arcLength();

    double pointsPerInch = 2;//TODO: add as parameter and tune

    double n = pathLength*pointsPerInch;

    for (int i = 0; i <= n; i++) {
        double t = (double)i / n;
        double v = trapezoidalVelocity(t, maxVel);
        waypoint wp = {{x(t), y(t), tangentAngle(t)}, v, v*curvature(t)*(tankWidth/2)};
        
        path.push_back(wp);
    }

    return path;

}

double TankControl::arcLength() {
    
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


double TankControl::trapezoidalVelocity(double t, double maxVel) {
     
    double maxAccel = 0.3; //TODO: add as parameter and tune
    t = std::clamp(t, 0.0, 1.0);

    double rf = std::clamp(maxVel / (2.0 * maxAccel), 0.0, 0.5);
    double peakVel = (rf < 0.5) ? maxVel : maxVel * (rf / 0.5);

    if (t < rf) {
        return peakVel * (t / rf);           // ramp up
    } else if (t <= 1.0 - rf) {
        return peakVel;                       // cruise
    } else {
        return peakVel * ((1.0 - t) / rf);  // ramp down
    }

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

Eigen::Matrix3d TankControl::solveRiccati(const Eigen::Matrix3d Ad,const Eigen::Matrix<double,3,2> Bd, const Eigen::Matrix3d Q, const Eigen::Matrix2d R, int iterations){
    
    Eigen::Matrix3d P = Q * 100.0;  // warm start larger
    Eigen::Matrix3d Pprev;

    for (int i = 0; i < 1000; ++i) {
        Eigen::Matrix2d S = R + Bd.transpose() * P * Bd;
        Pprev = P;
        P = Q + Ad.transpose() * P * Ad
              - Ad.transpose() * P * Bd
                * S.ldlt().solve(Bd.transpose() * P * Ad);

        // Check for convergence
        if ((P - Pprev).norm() < 1e-10) {
            break;
        }

        // Check for collapse
        if (!P.allFinite() || P.norm() < 1e-12) {
            std::cerr << "Riccati collapsed at iteration " << i << "\n";
            return Q;  // fallback: return Q so controller gets some gain
        }
    }
    return P;
}


double TankControl::tangentAngle(double t){
    return wrapHeading(atan2(dy(t), dx(t)));
}
double TankControl::x(double t){ 
    return pow((1-t),3)*p0.x + 3*pow((1-t),2)*t*p1.x + 3*(1-t)*pow(t,2)*p2.x + pow(t,3)*p3.x;
}
double TankControl::y(double t){
    return pow((1-t),3)*p0.y + 3*pow((1-t),2)*t*p1.y + 3*(1-t)*pow(t,2)*p2.y + pow(t,3)*p3.y;
}
double TankControl::dx(double t){
    return 3*pow(1-t,2)*(p1.x-p0.x) + 6*(1-t)*t*(p2.x-p1.x) + 3*pow(t,2)*(p3.x-p2.x);
}
double TankControl::dy(double t){
    return 3*pow(1-t,2)*(p1.y-p0.y) + 6*(1-t)*t*(p2.y-p1.y) + 3*pow(t,2)*(p3.y-p2.y);
}
double TankControl::ddx(double t){
    return -6*(1-t)*(p1.x-p0.x) + 6*(1-2*t)*(p2.x-p1.x) + 6*t*(p3.x-p2.x);
} 
double TankControl::ddy(double t){
    return -6*(1-t)*(p1.y-p0.y) + 6*(1-2*t)*(p2.y-p1.y) + 6*t*(p3.y-p2.y);
}
double TankControl::curvature(double t) {
    double num   = dx(t) * ddy(t) - dy(t) * ddx(t);
    double denom = std::pow(dx(t)*dx(t) + dy(t)*dy(t), 1.5);
    return (std::abs(denom) > 1e-9) ? num / denom : 0.0;
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
