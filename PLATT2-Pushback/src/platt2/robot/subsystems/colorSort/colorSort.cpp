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
                break;
        }
    }

    void ColorSortSubsystem::run() {
        optical_sensor->set_led_pwm(100);
    while(true){
    

    pros::c::optical_rgb_s_t currentRGB = optical_sensor->get_rgb();
    double currentR = currentRGB.red;
    double currentB = currentRGB.blue;

    switch(current_color)
    {
        case RED:
        {

         if(currentR > currentB)
         {
            intake_subsystem->tare_rear_motor_position();
            intake_subsystem->move_rear_motor(intake::OUT);

            while(intake_subsystem->get_rear_motor_position() < BALL_REJECTION_THRESHOLD) {
                pros::delay(10);
            }

            intake_subsystem->move_rear_motor(intake::STOP);
            break;
        }
        case BLUE:
        {
            if(currentB > currentR)
         {
            intake_subsystem->tare_rear_motor_position();
            intake_subsystem->move_rear_motor(intake::OUT);

            while(intake_subsystem->get_rear_motor_position() < BALL_REJECTION_THRESHOLD) {
                pros::delay(10);
            }

            intake_subsystem->move_rear_motor(intake::STOP);
            break;
        }
        case OFF:
        {
            break;
        }
    }
    }
    pros::delay(10);
    }
    }
}

}}}}