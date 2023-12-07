#ifndef ROBOTEAM_AI_RULESET_H
#define ROBOTEAM_AI_RULESET_H

#include <string>

namespace rtt::ai {

/**
 * @brief Structure that contains the ruleset of a certain game state
 */
struct RuleSet {
    /**
     * @brief Default constructor for the RuleSet structure
     */
    RuleSet() = default;

    /**
     * @brief Constructor for the RuleSet structure
     * @param title Title of the RuleSet
     * @param maxRobotVel Maximum allowed velocities for robots
     * @param robotsCanGoOutOfField Indicates whether the robots are allowed to go out of the field
     */
    RuleSet(std::string title, double maxRobotVel)
        : title(std::move(title)),
          maxRobotVel(maxRobotVel) {}

    std::string title;
    double maxRobotVel;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_RULESET_H
