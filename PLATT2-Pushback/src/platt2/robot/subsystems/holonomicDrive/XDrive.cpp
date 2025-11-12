#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "MovementVector.hpp"
#include "platt2/helperFunctions.h"
#include <algorithm>
#include "math.h"

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    XDrive::XDrive(std::vector<std::unique_ptr<XDriveModule>> drive_modules){
        this->drive_modules = (std::move(drive_modules));
    }

    void XDrive::moveVector(MovementVector v){
               
        double projection;
        double s;
    
        for (auto& module : drive_modules) {
            
            projection = -(cos(v.theta+module->getPhi())/cos(module->getTheta()));
    
            s = std::max(s, fabs(projection));
        }

        v.normalization_scalar = s/v.r;

        for (auto& module : drive_modules){
            module->move_vector(v);}
        }

    void XDrive::turnToHeading(double heading){
        for (auto& module : drive_modules) {
                
        }
    }
     
} // class

}}} // namespaces
