//
// Created by jesse on 14.10.19.
//

#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreeProtoType.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"

namespace bt {


    // Set the robottypes for the robot so the robotdealer can decide which robot should do what
    TreeProtoType::TreeProtoType(){
        this->robots = {
                {"o1", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o2", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o3", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o4", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o5", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o6", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o7", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o8", rtt::ai::robotDealer::RobotType::RANDOM}
        };
    }



std::shared_ptr<BehaviorTree> TreeProtoType::createOffensiveStrategy() {
    std::shared_ptr<Blackboard> bb = std::make_shared<Blackboard>();
    // Set the tactictype so the robotdealer can divide the robots
    // Currently there are 2 options: General and (Offensive/Defensive/Midfield).
    // This property is given to the blackboard, which is then given to the tactic. As far as @RobotJesse could tell,
    // this is only used in the tactic itself to determine which robots it should tick
    // in the updateStyle function of DefaultTactic.
    bb->setString("TacticType", "General");

    std::shared_ptr<DefaultTactic> defensiveTactic = createOffensiveTactic(bb);
    defensiveTactic->setProperties(bb);

    auto tree = std::make_shared<BehaviorTree>("defendertree");
    tree->SetRoot(defensiveTactic);
    return tree;
}

std::shared_ptr<DefaultTactic> TreeProtoType::createOffensiveTactic(std::shared_ptr<Blackboard> bb) {
    // create a default tactic which will be used to build the defensive tactic
    std::shared_ptr<DefaultTactic> defensiveTactic = std::make_shared<DefaultTactic>("defensiveTactic", bb, robots);

    // Creating the roles for all the robots in the tactic:
    for (int i = 1; i < robots.size(); i++) {
        std::string name = "o" + std::to_string(i);
        auto temp = createOffenderRole(name);
        std::shared_ptr<Role> temprole = createOffenderRole(name);
        temp->giveProperty("ROLE", name);
        defensiveTactic->addChild(temprole);
    }

    return defensiveTactic;
}

/// This function creates a defender role. This role consists of a repeater with as child an attack skill. It is important to remember that the order in which the nodes are
/// added is important.
std::shared_ptr<Role> TreeProtoType::createOffenderRole(std::string name) {
    // set the rolename for the current role. This is important because the robotdealer decides how to deal the robots based on their rolenames
    // localbb is the blackboard that is given to the attack skill. The property "ROLE" should be set to the name of the role, which should correspond to the rolename
    // found in the robotdealer robots vector
    auto localbb = std::make_shared<Blackboard>();
    localbb->setString("ROLE", name);
    std::shared_ptr<Role> role = std::make_shared<Role>(name);

    // Give the role the blackboard. This is used by the robotdealer to find the robot
    // TODO: figure out the exact difference between giving ROLE property to Role blackboard vs giving ROLE property to Skill blackboard
    // It seems ROLE just needs to be given to both. This is always safe. We need to implement a recursive role setter
    role->setProperties(localbb);

    // Make a repeater with the child "attack"
    std::shared_ptr<Repeater> repeater = std::make_shared<Repeater>();
    std::shared_ptr<rtt::ai::Attack> attack = std::make_shared<rtt::ai::Attack>("attack",  localbb);

    repeater->addChild(attack);
    role->addChild(repeater);
    return role;

}



}
