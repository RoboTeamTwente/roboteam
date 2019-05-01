#ifndef ROBOTEAM_AI_SHOTATGOAL_H
#define ROBOTEAM_AI_SHOTATGOAL_H

#include "Condition.h"
#include "roboteam_ai/src/control/PositionUtils.h"

namespace rtt {
namespace ai {

class HasClearShot : public Condition {
private:
    double minViewAtGoal = 0.20;
    const double MAX_SHOOTING_DISTANCE = 3.5;
public:
    explicit HasClearShot(std::string name = "HasClearShot", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_SHOTATGOAL_H
