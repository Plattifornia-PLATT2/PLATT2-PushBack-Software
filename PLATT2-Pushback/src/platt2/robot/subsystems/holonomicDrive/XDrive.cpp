#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "MovementVector.hpp"
#include <algorithm>

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    XDrive::XDrive(std::vector<std::unique_ptr<XDriveModule>> drive_modules){
        this->drive_modules = (std::move(drive_modules));
    }

    void XDrive::moveVector(MovementVector v){
        for (auto& module : drive_modules) {
                module->move_vector(v);
        }
    }

    void XDrive::turnToHeading(double heading){
        for (auto& module : drive_modules) {
                
        }
    }
     
} // class

}}} // namespaces
