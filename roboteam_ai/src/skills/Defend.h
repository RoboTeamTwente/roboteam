/*
 * Pick a nice defensive location,
 * drive towards there and keep between the ball and the goal.
 */

#ifndef ROBOTEAM_AI_DEFEND_H
#define ROBOTEAM_AI_DEFEND_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Defend : public Skill {

public:
    explicit Defend(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    bt::Node::Status onUpdate() override;
    void onTerminate(bt::Node::Status) override;
private:
    control::PositionController gtp;
    Vector2 targetLocation;
    static std::vector<std::shared_ptr<roboteam_msgs::WorldRobot>> allDefenders;
    Vector2 getDefensivePosition();
    int allDefendersMemory = 0;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_DEFEND_H
