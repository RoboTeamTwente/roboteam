#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"
<<<<<<< HEAD
#include "ros/ros.h"
=======
#include <ros/ros.h>
>>>>>>> Skills

namespace rtt {
namespace ai {

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
 class Skill : public bt::Leaf {
public:
    explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
