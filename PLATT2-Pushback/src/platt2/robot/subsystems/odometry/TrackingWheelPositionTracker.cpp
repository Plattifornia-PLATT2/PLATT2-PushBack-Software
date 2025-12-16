#include "platt2/robot/subsystems/odometry/TrackingWheelPositionTracker.hpp"

namespace platt2{
namespace robot{
namespace subsystems{
namespace odometry{

    OdometryPosition TrackingWheelPositionTracker::getPos(){
        return current_position;
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