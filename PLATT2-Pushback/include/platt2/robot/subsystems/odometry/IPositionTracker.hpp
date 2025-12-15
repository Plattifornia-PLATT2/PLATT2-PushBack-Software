#ifndef IPOSITION_TRACKER_HPP
#define IPOSITION_TRACKER_HPP

#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"

 /**
 * @brief Namespace for PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
* @brief Namespace for robot-related code.
* @authors PLATT2 Development team.
*/ 
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors Doiminic Young
 */    
namespace subsystems{

/**
 * @brief Namespace for odometry subsystem related code.
 * @authors Dominic Young & Logan Wolf
 */
namespace odometry{

    /**
    * @brief Interface for the tracking system of the robot.
    * @author Dominic Young
    */
    class IPositionTracker{
        public:
        virtual ~IPositionTracker() = default;

        /**
         * @brief Gets the current position of the robot with x, y, and heading values.
         * 
         * @return OdometryPosition Struct containing x, y, and heading of the robot.
         */
        virtual OdometryPosition getPos() = 0;

        /**
         * @brief Gets the current x position of the robot.
         * 
         * @return double The x position in inches.
         */
        virtual double getX() = 0;

        /**
         * @brief Gets the current y position of the robot.
         * 
         * @return double The y position in inches.
         */
        virtual double getY() = 0;

        /**
         * @brief Gets the current heading of the robot.
         * 
         * @return double The heading in degrees.
         */
        virtual double getHeading() = 0;

        /**
         * @brief Sets the offsets for the position tracker.
         * 
         * @param offsets The struct containing new offset values.
         */
        virtual void setOffsets(OdometryPosition offsets) = 0;
    };

}}}}
#endif