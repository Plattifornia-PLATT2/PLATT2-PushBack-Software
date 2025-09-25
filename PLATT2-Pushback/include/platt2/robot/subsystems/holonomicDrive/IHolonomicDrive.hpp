#ifndef IHOLONOMIC_HPP
#define IHOLONOMIC_HPP

#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    class IHolonomic {
        virtual void moveVector(MovementVector v);

        virtual void turnToHeading(double heading); 
    };


};
};
};
};

#endif