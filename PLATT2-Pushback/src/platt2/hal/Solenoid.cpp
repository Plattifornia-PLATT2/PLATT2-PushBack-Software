#include "platt2/hal/Solenoid.hpp"
#include "pros/adi.hpp"
#include <memory>

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

    Solenoid::Solenoid(u_int8_t port){
        solenoid = std::make_unique<pros::adi::DigitalOut>(port);
        currentState = false;
    }

    bool Solenoid::getState(){
        return currentState;
    }

    void Solenoid::setState(bool state){
        solenoid->set_value(state);
        currentState = state;
    }

    void Solenoid::toggleState(){
        solenoid->set_value(!currentState);
        currentState = !currentState;
    }

}}