#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/hal/Solenoid.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <memory>

namespace platt2{
namespace robot{
namespace subsystems{
namespace intake{

    
    void IntakeSubsystem::moveIntake(IntakeDirection direction){
        switch(direction){
            case IN:{
                front_intake_motor->move(NORMAL_INTAKE_VOLTAGE);
                left_matchload_motor->move(NORMAL_INTAKE_VOLTAGE);
                right_matchload_motor->move(NORMAL_INTAKE_VOLTAGE);
                break;
            }
            case OUT:{
                front_intake_motor->move(-NORMAL_INTAKE_VOLTAGE);
                left_matchload_motor->move(-NORMAL_INTAKE_VOLTAGE);
                right_matchload_motor->move(-NORMAL_INTAKE_VOLTAGE);
                break;
            }
            case OUT_LOW_GOAL:{
                front_intake_motor->move(LOW_GOAL_INTAKE_VOLTAGE);
                left_matchload_motor->move(LOW_GOAL_INTAKE_VOLTAGE);
                right_matchload_motor->move(LOW_GOAL_INTAKE_VOLTAGE);
                break;
            }
            case STOP:{
                front_intake_motor->move(0);
                left_matchload_motor->move(0);
                right_matchload_motor->move(0);
                break;
            }
        }
    }

    void IntakeSubsystem::toggleMatchloadPistons(){
        matchload_piston->toggleState();
    }

    bool IntakeSubsystem::getMatchloadPistonState(){
        return matchload_piston->getState();
    }
    
    void IntakeSubsystem::setFrontIntakeMotor(std::unique_ptr<pros::Motor> motor){
        front_intake_motor = std::move(motor);
    }

    void IntakeSubsystem::setLeftMatchloadMotor(std::unique_ptr<pros::Motor> motor){
        left_matchload_motor = std::move(motor);
    }

    void IntakeSubsystem::setRightMatchloadMotor(std::unique_ptr<pros::Motor> motor){
        right_matchload_motor = std::move(motor);
    }

    void IntakeSubsystem::setMatchloadPiston(std::unique_ptr<hal::Solenoid> piston){
        matchload_piston = std::move(piston);
    }

}
}
}
}