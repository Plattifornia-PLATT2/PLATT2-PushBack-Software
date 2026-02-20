#include "platt2\robot\subsystems\odometry\TriWheelPositionTracker.hpp"
#include "platt2/helperFunctions.h"
#include "OdometryPosition.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <iostream>
#include <array>

namespace platt2{
namespace robot{
namespace subsystems{
namespace odometry{

    OdometryPosition TriWheelPositionTracker::getPos(){
        return current_position;
    }

    void TriWheelPositionTracker::setPos(OdometryPosition pos){
        current_position.x = pos.x;
        current_position.y = pos.y;
        current_position.heading = pos.heading;
    }

    double TriWheelPositionTracker::getX(){
        return current_position.x;
    }

    double TriWheelPositionTracker::getY(){
        return current_position.y;
    }

    double TriWheelPositionTracker::getHeading(){
        return current_position.heading;
    }

    double TriWheelPositionTracker::getIMUHeading(){

        return 360 - imu->get_heading();

    }

    void TriWheelPositionTracker::setOffsets(double x_offset, double y_offset){
        this->x_offset = x_offset;
        this->y_offset = y_offset;
    }

    void TriWheelPositionTracker::init(){
        x_wheel->init();
        y1_wheel->init();
        y2_wheel->init();
        imu->reset();

        while(imu->is_calibrating()){
            pros::delay(10);
        }
    }

    void TriWheelPositionTracker::updatePosition(){

    std::array<double, 3> dTheta;
    std::array<double, 3> dPos;

    std::array<double, 3> w1 = x_wheel->getPlacment();
    std::array<double, 3> w2 = y1_wheel->getPlacment();
    std::array<double, 3> w3 = y2_wheel->getPlacment();

    std::array<std::array<double,3>,3> offsets{{  {cos(w1[2]), sin(w1[2]), (w1[0]*sin(w1[2])-w1[1]*cos(w1[2]))}, 
                                                  {cos(w2[2]), sin(w2[2]), (w2[0]*sin(w2[2])-w2[1]*cos(w2[2]))},
                                                  {cos(w3[2]), sin(w3[2]), (w3[0]*sin(w1[2])-w3[1]*cos(w3[2]))}
                                                }};
   
    std::array<std::array<double,3>,3> B = scale(inverse(offsets), 1/x_wheel->getR());
    
    std::array<double, 3> oldPos = {{x_wheel->getAngle(), y1_wheel->getAngle(), y2_wheel->getAngle()}};
    std::array<double, 3> newPos;

    while(true){

        newPos = {{x_wheel->getAngle(), y1_wheel->getAngle(), y2_wheel->getAngle()}};

        dTheta[0] = newPos[0] - oldPos[0];
        dTheta[1] = newPos[1] - oldPos[1];
        dTheta[2] = newPos[2] - oldPos[2];


        dPos = matrixMult(B, dTheta);

        current_position.x = current_position.x + dPos[0];
        current_position.y = current_position.y + dPos[1];
        current_position.heading = current_position.heading + dPos[2];
        
        
        oldPos = newPos;

        pros::delay(5);

    }

        
}

TriWheelPositionTracker::TriWheelPositionTracker(
            std::unique_ptr<hal::TrackingWheel> x_wheel, 
            std::unique_ptr<hal::TrackingWheel> y1_wheel, 
            std::unique_ptr<hal::TrackingWheel> y2_wheel,
            std::unique_ptr<pros::IMU> imu
        )
        : x_wheel(std::move(x_wheel)),
          y1_wheel(std::move(y1_wheel)),
          y2_wheel(std::move(y2_wheel)),
          imu(std::move(imu)) 
    {

    }
}
}
}
}
