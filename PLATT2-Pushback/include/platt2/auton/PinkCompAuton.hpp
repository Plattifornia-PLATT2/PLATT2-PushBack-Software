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

/**
 * @brief Namespace for all PLATT2 Library Code
 * @authors PLATT2 development team
 */
namespace platt2{

/**
 * @brief Namespace for all autonomous routines
 * @author Dominic Young
 */
namespace auton{

/**
 * @brief The pink robot competition autonomous routine implementation
 * @authors Brett Kucko, Quinn Smith, and Logan Wolf
 */
class PinkCompAuton : public auton::IAuton {
    private:
    std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem;
    std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem;
    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem;
    std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem;
    
    public:
    /**
     * @brief Initalizes the autonomous routine
     * 
     * @param holonomic_subsytem The holonomic subsystem to use
     * @param odometry_subsystem The odometry subsystem to use
     * @param intake_subsystem The intake subsystem to use
     * @param color_sort_subsystem The color sort subsystem to use
     */
    void init(
        std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem, 
        std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
        std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem,
        std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem  
    ) override; 

    /**
     * @brief Runs the autonomous routine
     * 
     */
    void run() override;
};

}}

#endif