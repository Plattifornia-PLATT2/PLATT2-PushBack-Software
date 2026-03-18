#include "platt2/robot/subsystems/lever/LeverSubsystem.hpp"
#include "pros/motors.hpp"
#include <memory>

namespace platt2{

/**
 * @brief Namespace for all hardware abstraction layer code.
 * @author Dominic Young
 */
namespace robot{

/**
 * @brief Namespace for all subsystem code
 * @author Dominic Young
 */
namespace subsystems{

/**
 * @brief Namespace for the lever subsystem code
 * @author Dominic Young
 */
namespace lever{

    void LeverSubsystem::setLeftMotor(std::unique_ptr<pros::Motor> motor){
        left_motor = std::move(motor);
    }

    void LeverSubsystem::setRightMotor(std::unique_ptr<pros::Motor> motor){
        right_motor = std::move(motor);
    }

    void LeverSubsystem::moveToPosition(LeverPositions position){
        switch(position){
            case MIDDLE:{
                left_motor->move_absolute(MIDDLE_LEVER_TRAVEL_SETPOINT,MIDDLE_LEVER_VOLTAGE);
                right_motor->move_absolute(MIDDLE_LEVER_TRAVEL_SETPOINT,MIDDLE_LEVER_VOLTAGE);
                break;
            }
            case LONG:{
                left_motor->move_absolute(LONG_LEVER_TRAVEL_SETPOINT,LONG_LEVER_VOLTAGE);
                right_motor->move_absolute(LONG_LEVER_TRAVEL_SETPOINT,LONG_LEVER_VOLTAGE);
                break;
            }
            case HOME:{
                left_motor->move_absolute(HOME_LEVER_TRAVEL_SETPOINT,LONG_LEVER_VOLTAGE);
                right_motor->move_absolute(HOME_LEVER_TRAVEL_SETPOINT,LONG_LEVER_VOLTAGE);
                break;
            }
        }
    }

}}}}