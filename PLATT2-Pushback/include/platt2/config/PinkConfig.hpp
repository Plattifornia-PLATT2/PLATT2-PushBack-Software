#ifndef PINKCONFIG_HPP
#define PINKCONFIG_HPP

#include "IConfig.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include <memory>
namespace platt2{

namespace config{

class PinkConfig : public IConfig {
    private:
    static constexpr pros::MotorGears DRIVE_GEARSET{pros::MotorGears::blue};

    static constexpr int LEFT_FRONT_MODULE_TOP_PORT{1};
    static constexpr int LEFT_FRONT_MODULE_BOTTOM_PORT{-2};

    static constexpr int RIGHT_FRONT_MODULE_TOP_PORT{-3};
    static constexpr int RIGHT_FRONT_MODULE_BOTTOM_PORT{4};

    static constexpr int LEFT_REAR_MODULE_TOP_PORT{5};
    static constexpr int LEFT_REAR_MODULE_BOTTOM_PORT{-6};

    static constexpr int RIGHT_REAR_MODULE_TOP_PORT{-9};
    static constexpr int RIGHT_REAR_MODULE_BOTTOM_PORT{8};

    static constexpr int FRONT_INTAKE_MOTOR_PORT{10};
    static constexpr int MIDDLE_INTAKE_MOTOR_PORT{11};
    static constexpr int REAR_INTAKE_MOTOR_PORT{12};
    static constexpr int UPPER_CONVEYOR_MOTOR_PORT{13};
    static constexpr pros::MotorGears INTAKE_GEARSET{pros::MotorGears::blue};

    static constexpr double position_dt{1};
    static constexpr double position_max{1};
    static constexpr double position_min{1};
    static constexpr double position_Kp{1};
    static constexpr double position_Kd{1};
    static constexpr double position_Ki{1};

    static constexpr double heading_dt{0.01};
    static constexpr double heading_max{0.4};
    static constexpr double heading_min{-0.4};
    static constexpr double heading_Kp{0.5};
    static constexpr double heading_Kd{0};
    static constexpr double heading_Ki{0};

    public:

    std::shared_ptr<robot::Robot> buildRobot() override;
};
}
}

#endif