#ifndef XDRIVE_HPP
#define XDRIVE_HPP

#include "platt2/robot/subsystems/holonomicDrive/IHolonomicDrive.hpp"
#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"

#include <memory>
#include <vector>

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

class XDrive : public IHolonomic {
    private:
    std::vector<std::unique_ptr<XDriveModule>> drive_modules; 

    public:
        void moveVector(MovementVector v);

        void turnToHeading(double heading);

        XDrive(std::vector<std::unique_ptr<XDriveModule>> drive_modules);
};

}
}
}
}

#endif