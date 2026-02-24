#include "platt2/robot/subsystems/odometry/TrackingWheelPositionTracker.hpp"
#include "OdometryPosition.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <iostream>

namespace platt2{
namespace robot{
namespace subsystems{
namespace odometry{

    OdometryPosition TrackingWheelPositionTracker::getPos(){
        return current_position;
    }

    void TrackingWheelPositionTracker::setPos(OdometryPosition pos){
        current_position.x = pos.x;
        current_position.y = pos.y;
        current_position.heading = pos.heading*(M_PI/180);
        imu->set_heading(360 - pos.heading);
    }

    double TrackingWheelPositionTracker::getX(){
        return current_position.x;
    }

    double TrackingWheelPositionTracker::getY(){
        return current_position.y;
    }

    double TrackingWheelPositionTracker::getHeading(){
        return current_position.heading;
    }

    void TrackingWheelPositionTracker::setOffsets(double x_offset, double y_offset){
        this->x_offset = x_offset;
        this->y_offset = y_offset;
    }

    void TrackingWheelPositionTracker::init(){
        x_wheel->init();
        y_wheel->init();
        imu->reset();

        while(imu->is_calibrating()){
            pros::delay(10);
        }
    }

    void TrackingWheelPositionTracker::updatePosition(){
            
        //init variables
        std::cout << current_position.heading << std::endl;
        current_position.heading  = ((360 - imu->get_heading())*M_PI/180);
        double oldTheta = current_position.heading ;
        double dTheta = (current_position.heading  - oldTheta);

        double newX = x_wheel->getPosition();
        double oldX = newX;
        double dX = newX - oldX;

        double newY = y_wheel->getPosition();
        double oldY = newY;
        double dY = newY - oldY;

        double localX = 0;
        double localY = 0;

        double globalX = 0;
        double globalY = 0;

        
        while(true){

            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1,"X Pos: %.3f", current_position.x);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2,"Y Pos: %.3f", current_position.y);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3,"Heading: %.2f", current_position.heading*180/M_PI);


            newX = x_wheel->getPosition();
            newY = y_wheel->getPosition();
            current_position.heading  = ((360 - imu->get_heading())*M_PI/180);

            dX = newX - oldX;
            dY = newY - oldY;
            dTheta = current_position.heading - oldTheta;

            if(dTheta > M_PI || dTheta < -M_PI){
                dTheta = -1 * sgn(dTheta) * (2*M_PI - std::abs(dTheta));
            }

            oldX = newX;
            oldY = newY;
            oldTheta = current_position.heading;

            localX = dX - (x_offset*dTheta);
            localY = dY + (y_offset*dTheta);


            localX = (1-(pow(dTheta, 2)/24))*localX;
            localY = (1-(pow(dTheta, 2)/24))*localY;

            globalX =  localX * sin(oldTheta+dTheta/2)
                        + localY * cos(oldTheta+dTheta/2);

            globalY =  localX * cos(oldTheta+dTheta/2)
                        - localY * sin(oldTheta+dTheta/2);

            if (std::isnan(globalX) or std::isnan(globalY)){
                pros::delay(10);
                continue;
            }

            current_position.x += globalX;
            current_position.y += globalY;


            pros::delay(10);
        }
    }

    TrackingWheelPositionTracker::TrackingWheelPositionTracker(
        std::unique_ptr<hal::TrackingWheel> x_wheel,
        std::unique_ptr<hal::TrackingWheel> y_wheel,
        std::unique_ptr<pros::IMU> imu
    )
        : x_wheel(std::move(x_wheel)),
          y_wheel(std::move(y_wheel)),
          imu(std::move(imu)) 
    {

    }

}}}}