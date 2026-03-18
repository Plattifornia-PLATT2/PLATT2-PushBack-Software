#include "pros/adi.hpp"
#include <memory>
#include <sys/types.h>

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for all hardware abstraction layer code.
 * @author Dominic Young
 */
namespace hal{

    /**
     * @brief Class to represent a Pneumatic Solenoid with expanded functionality over the pros::adi class.
     * @author Dominic Young
     */
    class Solenoid{
        private: 
            /**
             * @brief A pointer to the ADI port digital out to control the solenoid
             * 
             */
            std::unique_ptr<pros::adi::DigitalOut> solenoid;

            /**
             * @brief The current state of the solenoid. TRUE for HIGH, FALSE for LOW
             * 
             */
            bool currentState;

        public:

            /**
             * @brief Construct a new Solenoid object.
             * 
             * @param portNumber V5 ADI Port the solenoid is used in (A-H).
             */
            Solenoid(u_int8_t port);

            /**
             * @brief Gets the state of the solenoid.
             * 
             * @return true when the solenoid has a HIGH state (Piston is extended).
             * @return false when the solenoid has a LOW state (Piston is retracted).
             */
            bool getState();

            /**
             * @brief Set the state of the solenoid
             * 
             * @param state The new state to set.
             */
            void setState(bool state);

            /**
             * @brief Toggles the state of the solenoid.
             * 
             */
            void toggleState();

    };
}
}