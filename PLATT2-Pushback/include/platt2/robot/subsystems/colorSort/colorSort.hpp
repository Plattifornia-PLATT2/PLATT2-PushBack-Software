#ifndef COLORSORT_HPP
#define COLORSORT_HPP

#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <memory>
namespace platt2{
namespace robot{
namespace subsystems{
namespace colorsort{

    enum BallColor{
        OFF = 0,
        RED = 1,
        BLUE = 2
    };

    class ColorSortSubsystem{
        private:
        std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem;
        std::unique_ptr<pros::Optical> optical_sensor;

        BallColor current_color = OFF;
        double BALL_REJECTION_THRESHOLD = 10;

        pros::Task color_sort_task;
        public:
        
        void setSortedColor(BallColor color);

        void run();

        ColorSortSubsystem(
            std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem,
            std::unique_ptr<pros::Optical> optical_sensor
        );
    };
}
}
}
}

#endif