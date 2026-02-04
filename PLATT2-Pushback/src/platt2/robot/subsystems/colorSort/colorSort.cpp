#include "platt2/robot/subsystems/colorSort/colorSort.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace robot{
namespace subsystems{
namespace colorsort{

    static void taskFunction(void *p) {
        reinterpret_cast<ColorSortSubsystem*>(p)->run();
    }

    ColorSortSubsystem::ColorSortSubsystem(
        std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem,
        std::unique_ptr<pros::Optical> optical_sensor
    ) : 
    intake_subsystem(intake_subsystem), 
    optical_sensor(std::move(optical_sensor)),
    color_sort_task(taskFunction, this)
    {

    }

    void ColorSortSubsystem::setSortedColor(BallColor color) {
        current_color = color;
    }

    BallColor ColorSortSubsystem::getSortedColor(){
        return current_color;
    }

    bool ColorSortSubsystem::isSortActive(){
        return isActive;
    }

    void ColorSortSubsystem::toggleSortedColor() {
        switch(current_color) {
            case OFF:
                current_color = RED;
                break;
            case RED:
                current_color = BLUE;
                break;
            case BLUE:
                current_color = OFF;
                isActive = false;
                break;
        }
    }

    void ColorSortSubsystem::run() {
        optical_sensor->set_led_pwm(100);
        while(true){
    

        pros::c::optical_rgb_s_t currentRGB = optical_sensor->get_rgb();
        double currentR = currentRGB.red;
        double currentB = currentRGB.blue;
        double currentDistance = optical_sensor->get_proximity();
        //std::cout<<"R: "<<currentR<<" B: "<<currentB<<" Distance: "<<currentDistance<<std::endl;

        switch(current_color)
        {
            case RED:
            {

            if(currentR > currentB && currentDistance > BALL_DISTANCE_THRESHOLD)
            {
                isActive = true;
                //std::cout<<"r>b"<<std::endl;
                intake_subsystem->tare_rear_motor_position();
                intake_subsystem->move_rear_motor(intake::OUT);
                //std::cout<<intake_subsystem->get_rear_motor_position()<<std::endl;

                while(intake_subsystem->get_rear_motor_position() > BALL_REJECTION_THRESHOLD) {
                    std::cout<<intake_subsystem->get_rear_motor_position()<<std::endl;
                    pros::delay(10);
                }

            intake_subsystem->move_rear_motor(intake::STOP);
            }
            break;
            }
            case BLUE:
            {
                if(currentB > currentR && currentDistance > BALL_DISTANCE_THRESHOLD)
                {
                isActive = true;
                //std::cout<<"b>r"<<std::endl;
                intake_subsystem->tare_rear_motor_position();
                intake_subsystem->move_rear_motor(intake::OUT);

                while(intake_subsystem->get_rear_motor_position() > BALL_REJECTION_THRESHOLD) {
                    pros::delay(10);
                }

                intake_subsystem->move_rear_motor(intake::STOP);
                break;
                    }
                }
            case OFF:
            {
                isActive = false;
                break;
            }
        }
                isActive = false;
                pros::delay(10);
    }

        }       
}}}}