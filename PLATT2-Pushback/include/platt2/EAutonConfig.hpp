#ifndef EAUTONCONFIG_HPP
#define EAUTONCONFIG_HPP

/**
 * @brief The namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief The namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{
    
/**
 * @brief Enumeration to represent the 6 different autonomous modes the robot may operate in with a non auton option.
 * @author Dominic Young
 */
 enum AutonConfig
 {
 
     NO_AUTON = 0,
     COMP_1 = 1,
     SKILLS_1 = 2
 
 };
}
}

#endif