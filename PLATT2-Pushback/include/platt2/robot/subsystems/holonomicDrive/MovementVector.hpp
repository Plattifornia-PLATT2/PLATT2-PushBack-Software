#ifndef MOVEMENTVECTOR_HPP
#define MOVEMENTVECTOR_HPP

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    struct MovementVector {
    double r;
    double theta;
    double w;
    double normalization_scalar;  
};
}
}
}
}
#endif