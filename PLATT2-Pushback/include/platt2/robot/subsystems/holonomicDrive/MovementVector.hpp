#ifndef MOVEMENTVECTOR_HPP
#define MOVEMENTVECTOR_HPP

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    struct MovementVector {
    double travel_angle;
    double linear_speed;
    double target_heading;
    double current_heading;
    double normalization_scalar = 1.0;  // added
};
}
}
}
}
#endif