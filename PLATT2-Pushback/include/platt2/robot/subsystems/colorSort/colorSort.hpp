#ifndef COLORSORT_HPP
#define COLORSORT_HPP

#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <memory>
namespace platt2{
namespace robot{
namespace subsystems{
namespace colorsort{

    

    class ColorSortSubsystem{
        private:
        std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem;
        std::unique_ptr<pros::Optical> optical_sensor;

        public:

    };
}
}
}
}

#endif