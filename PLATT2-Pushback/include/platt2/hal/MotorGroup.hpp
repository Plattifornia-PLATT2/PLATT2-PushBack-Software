#ifndef MOTOR_GROUP_HPP
#define MOTOR_GROUP_HPP

#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include <memory>
#include <vector>

namespace platt2{

namespace hal{

/// @brief Class to use a group of motors
/// @author Dominic Young
class MotorGroup{
    private:
    std::vector<std::unique_ptr<pros::Motor>> motors {};

    public:
    void setVoltage(double voltage);

    void setVelocity(double velocity);

    void addMotor(std::unique_ptr<pros::Motor> &motor);

    void setMotorGearing(pros::MotorGear gears);

    void setZeroPostion();

    double getVoltage();

    double getVelocity();

    pros::Motor& getMotor(int index);

    MotorGroup& operator=(MotorGroup& rhs);
}; 
} // namespace hal
} // namespace platt2

#endif