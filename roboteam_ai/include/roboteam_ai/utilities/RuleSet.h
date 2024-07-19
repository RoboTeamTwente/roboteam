#ifndef ROBOTEAM_AI_RULESET_H
#define ROBOTEAM_AI_RULESET_H

#include <string>

enum RuleSetName { DEFAULT, HALT, STOP };

namespace rtt::ai {

/**
 * @brief Structure that contains the ruleset of a certain game state
 */
struct RuleSet {
   public:
    /**
     * @brief Default constructor for the RuleSet structure
     */
    RuleSet() = default;

    /**
     * @brief Constructor for the RuleSet structure
     * @param title Title of the RuleSet
     * @param maxRobotVel Maximum allowed velocities for robots
     */
    constexpr RuleSet(RuleSetName title, double maxRobotVel) : title(std::move(title)), maxRobotVel(maxRobotVel) {}

    /**
     * @brief Getter for the title of the RuleSet
     * @return Title of the RuleSet
     */
    RuleSetName getTitle() const { return title; }

    /**
     * @brief Getter for the maximum allowed velocities for robots
     * @return Maximum allowed velocities for robots
     */
    double getMaxRobotVel() const { return maxRobotVel; }

    std::string toString() const {
        switch (title) {
            case RuleSetName::DEFAULT:
                return "Default";
            case RuleSetName::HALT:
                return "Halt";
            case RuleSetName::STOP:
                return "Stop";
            default:
                return "Unknown";
        }
    }

    static constexpr RuleSet RULESET_DEFAULT() { return {RuleSetName::DEFAULT, 2.0}; }
    static constexpr RuleSet RULESET_HALT() { return {RuleSetName::HALT, 0.0}; }
    static constexpr RuleSet RULESET_STOP() { return {RuleSetName::STOP, 1.0}; }

    static constexpr std::array<RuleSet, 3> ruleSets() {
        return {
            RULESET_DEFAULT(),
            RULESET_HALT(),
            RULESET_STOP(),
        };
    }

   private:
    RuleSetName title;
    double maxRobotVel;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_RULESET_H