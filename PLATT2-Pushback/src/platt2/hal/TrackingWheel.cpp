#include "platt2/hal/TrackingWheel.hpp"
#include <array>
#include <math.h>
#include <numbers>

namespace platt2{
namespace hal{

double TrackingWheel::getPosition(){
    return ((double)(rotation_sensor->get_position() / (double)CENTIDEGREES_CONVERSION_FACTOR) * wheel_circumference);
}
double TrackingWheel::getAngle(){
    return (((double)(rotation_sensor->get_position() / (double)CENTIDEGREES_CONVERSION_FACTOR))*2*M_PI);
}

double TrackingWheel::getR(){
    return wheel_diameter/2;
}

void TrackingWheel::setPosition(double pos){
    double rotations = pos / wheel_circumference;
    rotation_sensor->set_position(rotations / CENTIDEGREES_CONVERSION_FACTOR);
}

void TrackingWheel::init(){
    rotation_sensor->set_data_rate(5);
    rotation_sensor->reset();
    
}

std::array<double, 3> TrackingWheel::getPlacment(){

    return  this->placmentPos;

}

void TrackingWheel::setPlacment(std::array<double, 3> place){

    this->placmentPos = place;

}

TrackingWheel::TrackingWheel(std::unique_ptr<pros::Rotation> rot_sensor, double wheel_diameter)
: 
rotation_sensor(std::move(rot_sensor)), 
wheel_diameter(wheel_diameter) 
{
    wheel_circumference = wheel_diameter * M_PI;   
}    

}} 