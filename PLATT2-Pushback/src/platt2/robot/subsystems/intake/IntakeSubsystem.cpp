#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace robot{
namespace subsystems{
namespace intake{

    IntakeSubsystem::IntakeSubsystem(std::unique_ptr<pros::Motor>front_intake, std::unique_ptr<pros::Motor>rear_intake, std::unique_ptr<pros::Motor>mid_intake, std::unique_ptr<pros::Motor>upper_conveyor){
        front_intake_motor = std::move(front_intake);
        rear_intake_motor = std::move(rear_intake);
        middle_intake_motor = std::move(mid_intake);
        upper_conveyor_motor = std::move(upper_conveyor);
    }

    void IntakeSubsystem::move_intake(IntakeDirection direction){
        if(direction == IN){
            front_intake_motor->move_velocity(intake_speed);
            middle_intake_motor->move_velocity(intake_speed);
            rear_intake_motor->move_velocity(intake_speed);
            upper_conveyor_motor->move_velocity(intake_speed);
        }
        else if (direction == OUT){
            front_intake_motor->move_velocity(-intake_speed);
            middle_intake_motor->move_velocity(-intake_speed);
            rear_intake_motor->move_velocity(-intake_speed);
            upper_conveyor_motor->move_velocity(-intake_speed);   
        }
        else if (direction == STOP){
            front_intake_motor->move_velocity(0);
            middle_intake_motor->move_velocity(0);
            rear_intake_motor->move_velocity(0);
            upper_conveyor_motor->move_velocity(0);
        }
    }
}
}
}
}