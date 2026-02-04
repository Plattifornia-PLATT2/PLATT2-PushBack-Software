#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/helperFunctions.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include <fstream>


namespace platt2{
namespace robot{
namespace subsystems{
namespace holonomicDrive{

void HolonomicControl::moveToPoint(double x_target, double y_target, double target_heading, double rSpeed, double wSpeed, double timeout) {

    target_heading = target_heading*M_PI/180;

    MovementVector motionVector;
    polar p;

    double x_error;
    double y_error;
    double angle_error;

    std::vector<double> rArray(100,1);
    std::vector<double> wArray(100,1);

    avg rAve;
    avg wAve;

    double startTime = pros::millis();

    //std::ofstream file("/usd/data.csv", std::ios::app); // Creates a new file named "data.csv"

    double startPosX = odometry->getX();
    double startPosY = odometry->getY();

    double dev;
    double accel;
    double v2;

    double startLoopTime = pros::millis();


    while (true){
        
        x_error = x_target - odometry->getX();
        y_error = y_target - odometry->getY();

        p = CtoP(x_error, y_error);

        angle_error = target_heading - (odometry->getHeading()*M_PI/180);

        if(angle_error > M_PI || angle_error < -M_PI){
            angle_error = -1 * sgn(angle_error) * (2*M_PI - std::abs(angle_error));
        }
        std::cout<<angle_error<<std::endl;
        motionVector.r = std::clamp(-1*(positionPID->calculate(0, p.r)), -rSpeed, rSpeed);
        motionVector.theta = p.theta - (odometry->getHeading()*M_PI/180)+(M_PI/2);

        motionVector.w = std::clamp(headingPID->calculate(0, angle_error), -rSpeed, rSpeed);

        rAve = rollAverage(std::abs(motionVector.r), rArray);
        wAve = rollAverage(std::abs(motionVector.w), wArray);
        rArray = rAve.data;
        wArray = wAve.data;

        if (rAve.average < 0.05 && wAve.average < 00.05){break;}
        if (pros::millis()-startTime>timeout*1000){break;}
        
        drivetrain->moveVector(motionVector);
        pros::delay(10);

        dev = distanceFromSecant(startPosX, startPosY, x_target, y_target, odometry->getX(), odometry->getY());
        accel = motionVector.r-v2;
        v2 = motionVector.r;
        //file << angle_error << "," << x_error << "," << y_error << "," << dev << "," << accel << "," << motionVector.r << "," << motionVector.theta << "," << motionVector.w << ",0\n";
        startLoopTime = pros::millis();
    } 
    //file.close();
    //std::cout<<"I got to loop end"<<std::endl;
    motionVector.r = 0;
    motionVector.w = 0;
    motionVector.theta = 0;
    drivetrain->moveVector(motionVector);

}

void HolonomicControl::moveToPointInternalHeading(double x_target, double y_target, double target_heading, double rSpeed, double wSpeed, double timeout) {

    target_heading = target_heading*M_PI/180;

    MovementVector motionVector;
    polar p;

    double x_error;
    double y_error;
    double angle_error;

    std::vector<double> rArray(100,1);
    std::vector<double> wArray(100,1);

    avg rAve;
    avg wAve;

    double startTime = pros::millis();

    std::ofstream file("/usd/data.csv", std::ios::app); // Creates a new file named "data.csv"

    double startPosX = odometry->getX();
    double startPosY = odometry->getY();

    double dev;
    double accel;
    double v2;

    double startLoopTime = pros::millis();


    while (true){
        
        x_error = x_target - odometry->getX();
        y_error = y_target - odometry->getY();

        p = CtoP(x_error, y_error);

        angle_error = target_heading - odometry->getVexHeading();

        if(angle_error > M_PI || angle_error < -M_PI){
            angle_error = -1 * sgn(angle_error) * (2*M_PI - std::abs(angle_error));
        }
        
        motionVector.r = std::clamp(-1*(positionPID->calculate(0, p.r)), -rSpeed, rSpeed);
        motionVector.theta = p.theta - odometry->getVexHeading()+(M_PI/2);
        motionVector.w = std::clamp(headingPID->calculate(0, angle_error), -rSpeed, rSpeed);

        rAve = rollAverage(std::abs(motionVector.r), rArray);
        wAve = rollAverage(std::abs(motionVector.w), wArray);
        rArray = rAve.data;
        wArray = wAve.data;

        if (rAve.average < 0.05 && wAve.average < 00.05){break;}
        if (pros::millis()-startTime>timeout*1000){break;}
        
        drivetrain->moveVector(motionVector);
        pros::delay(10);

        dev = distanceFromSecant(startPosX, startPosY, x_target, y_target, odometry->getX(), odometry->getY());
        accel = motionVector.r-v2;
        v2 = motionVector.r;
        //file << angle_error << "," << x_error << "," << y_error << "," << dev << "," << accel << "," << motionVector.r << "," << motionVector.theta << "," << motionVector.w << ",0\n";
        startLoopTime = pros::millis();
    } 
    file.close();
    //std::cout<<"I got to loop end"<<std::endl;
    motionVector.r = 0;
    motionVector.w = 0;
    motionVector.theta = 0;
    drivetrain->moveVector(motionVector);

}

void HolonomicControl::staticTest(){

    MovementVector motionVector;

    motionVector.w = 0.01;



    drivetrain->moveVector(motionVector);

    pros::delay(1000);

    motionVector.w = 0;
    
    drivetrain->moveVector(motionVector);




}



HolonomicControl::HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID)
{
    drivetrain = drive;
    odometry = odom;
    positionPID = std::move(posPID);
    this->headingPID = std::move(headingPID);
}

}
}
}
}