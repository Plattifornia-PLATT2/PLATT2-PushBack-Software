#include "platt2/robot/subsystems/odometry/TrackingWheelPositionTracker.hpp"
#include "OdometryPosition.hpp"

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
        current_position.heading = pos.heading;
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

    void TrackingWheelPositionTracker::setOffsets(double x_offset, double y_offset, double heading_offset){
        this->x_offset = x_offset;
        this->y_offset = y_offset;
        this->heading_offset = heading_offset;
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
    current_position.heading  = 360 - imu->get_heading();
    double oldHeading = current_position.heading ;
    double deltaTheta = (current_position.heading  - oldHeading) * (M_PI/180);

    double hWheel = x_wheel->getPosition();
    double oldHWheel = hWheel;
    double deltaHWheel = hWheel - oldHWheel;

    double vWheel = y_wheel->getPosition();
    double oldVWheel = vWheel;
    double deltaVWheel = vWheel - oldVWheel;

    double deltaX = 0;
    double deltaY = 0;

    double hyp = 0;
    double theta = 0;
    double points[2] = {0,0};
    double zero[2] = {0,0};

    while(true){

        //update variables each iteration of loop
        current_position.heading  = 360 - imu->get_heading();
        deltaTheta = (current_position.heading  - oldHeading) * (M_PI/180);
        hWheel = x_wheel->getPosition();
        deltaHWheel = hWheel - oldHWheel;
        vWheel = y_wheel->getPosition();
        deltaVWheel = vWheel - oldVWheel;

        //deltaTheta = 0;

        //determins if the angle has changed and does relevent math 
        if(deltaTheta == 0){
           
            points[0] = deltaHWheel;
            points[1] = deltaVWheel;
        
        }else{
 
            points[0] = ((2*sin(deltaTheta/(2)))*((deltaHWheel/deltaTheta)+x_offset));
            points[1] = ((2*sin(deltaTheta/(2)))*((deltaVWheel/deltaTheta)+y_offset));

        }

        //convert local vector to the global oriantation
        hyp = pythag(zero, points);

        if (hyp < 0.001){hyp = 0;}

        if (deltaHWheel == 0){

            if(deltaVWheel != 0){

                if(deltaVWheel > 0){

                    theta = 90;

                }else{

                    theta = 270;

                }
            }

                theta = theta*(M_PI/180) - ((90-current_position.heading)*(M_PI/180) + (deltaTheta/2));

        }else{

            theta = atan2(points[1],points[0]) - ((90-current_position.heading)*(M_PI/180) + (deltaTheta/2));
                
        }

        deltaX = hyp*sin(theta);
        deltaY = hyp*cos(theta);

        //update poition using calculated vector
        current_position.x = current_position.x + deltaX;
        current_position.y = current_position.y + deltaY;
     
        //update variable for next loop
        oldHeading = current_position.heading;
        oldHWheel = hWheel;
        oldVWheel = vWheel;

        current_position.heading = (current_position.heading-180) + 180;

        if (current_position.heading<0){
            current_position.heading += 360;
        }

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