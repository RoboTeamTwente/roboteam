//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_RULESET_H
#define ROBOTEAM_AI_RULESET_H

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
     * @param maxBallVel Maximum allowed velocity for the ball
     * @param minDistanceToBall Minimum distance to the ball the robots have to keep
     * @param minDistanceToDefenseArea Minimum distance to the defense area the robots have to keep
     * @param robotsCanGoOutOfField Indicates whether the robots are allowed to go out of the field
     */
    RuleSet(std::string title, double maxRobotVel, double maxBallVel, double minDistanceToBall, double minDistanceToDefenseArea, bool robotsCanGoOutOfField)
        : title(std::move(title)),
          maxRobotVel(maxRobotVel),
          maxBallVel(maxBallVel),
          minDistanceToBall(minDistanceToBall),
          minDistanceToDefenseArea(minDistanceToDefenseArea),
          robotsCanGoOutOfField(robotsCanGoOutOfField) {}

    std::string title;
    double maxRobotVel;
    double maxBallVel;
    double minDistanceToBall;
    double minDistanceToDefenseArea;
    bool robotsCanGoOutOfField;

    /**
     * @brief Checks whether the robots are allowed to enter the defense area
     * @return Boolean that tells whether the robots are allowed to enter the defense area
     */
    bool robotsCanEnterDefenseArea() { return minDistanceToDefenseArea == -1; }
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_RULESET_H
