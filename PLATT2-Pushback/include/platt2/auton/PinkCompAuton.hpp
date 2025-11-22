#ifndef PINKCOMPAUTON_HPP
#define PINKCOMPAUTON_HPP

#include "IAuton.hpp"
#include "platt2/robot/Robot.hpp"
#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/subsystems/holonomicDrive/IHolonomicDrive.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include <memory>

namespace platt2{
namespace auton{

class PinkCompAuton : public auton::IAuton {
    private:
    std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem;
    std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem;
    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem;
    
    public:
    void init(
        std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem, 
        std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
        std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem
    ) override; 

    void run() override;
};

}}

#endif