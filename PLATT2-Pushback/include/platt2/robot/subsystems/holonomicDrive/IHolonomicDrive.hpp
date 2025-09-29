#ifndef IHOLONOMIC_HPP
#define IHOLONOMIC_HPP

#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    class IHolonomic {
        public:
        virtual ~IHolonomic() = default;

        virtual void moveVector(MovementVector v) = 0;

        virtual void turnToHeading(double heading) = 0; 
    };


};
};
};
};

#endif