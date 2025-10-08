#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "MovementVector.hpp"
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
    double v_x = v.linear_speed * std::cos(v.travel_angle);
    double v_y = v.linear_speed * std::sin(v.travel_angle);

    double maxVal = 1.0;
    for (auto& module : drive_modules) {
        double projection = v_x * std::cos(module->getAngleFromZero()) +
                            v_y * std::sin(module->getAngleFromZero());
        maxVal = std::max(maxVal, fabs(projection));
    }

    v.normalization_scalar = 1.0 / maxVal;

    for (auto& module : drive_modules){
        module->move_vector(v);}

    }

    void XDrive::turnToHeading(double heading){
        for (auto& module : drive_modules) {
                
        }
    }
     
} // class

}}} // namespaces
