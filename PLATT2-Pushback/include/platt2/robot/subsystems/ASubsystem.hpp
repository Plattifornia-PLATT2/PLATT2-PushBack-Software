#ifndef ASUBSYSTEM_HPP
#define ASUBSYSTEM_HPP

#include "platt2/robot/subsystems/ESubsystemCommands.hpp"
#include "platt2/robot/subsystems/ESubsystems.hpp"

namespace platt2{

namespace robot{

namespace subsystems{

    class ASubsystem{

        public:
        virtual ~ASubsystem() = default;

        virtual void run() = 0;
        virtual void init() = 0;
        virtual void command(SubsystemCommands command) = 0;
        
    };
}
}
}

#endif