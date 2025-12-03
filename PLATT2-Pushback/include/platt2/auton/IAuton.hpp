#ifndef IAUTON_HPP
#define IAUTON_HPP

#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/subsystems/holonomicDrive/IHolonomicDrive.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/colorsort/ColorSort.hpp"
#include <memory>

namespace platt2{
namespace auton{

/**
    @brief Interface for autonomous routines.
    @author Dominic Young
*/
class IAuton{
    public:

    virtual ~IAuton() = default;

    virtual void init(std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem, 
        std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
        std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem,
        std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem
    ) = 0;

    virtual void run() = 0;

};

}}

#endif