#include "stp/plays/referee_specific/PenaltyThemPrepare.h"

#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

// The x position on which the enemy takes the penalty
constexpr double PENALTY_MARK_THEM_X = 2.0;

PenaltyThemPrepare::PenaltyThemPrepare() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::PenaltyThemPrepareGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::PenaltyThemPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::Formation>("formation_0"),
        std::make_unique<role::Formation>("formation_1"),
        std::make_unique<role::Formation>("formation_2"),
        std::make_unique<role::Formation>("formation_3"),
        std::make_unique<role::Formation>("formation_4"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("formation_5"),
        std::make_unique<role::Formation>("formation_6"),
        std::make_unique<role::Formation>("formation_7"),
        std::make_unique<role::Formation>("formation_8"),
        std::make_unique<role::Formation>("formation_9"),
    };
}

uint8_t PenaltyThemPrepare::score(const rtt::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::PenaltyThemPrepareGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

Dealer::FlagMap PenaltyThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_9", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void PenaltyThemPrepare::calculateInfoForRoles() noexcept {
    PositionComputations::calculateInfoForKeeper(stpInfos, field, world);

    // During their penalty, all our robots should be behind the ball to not interfere.
    // Create a grid pattern of robots on their side of the field
    int amountOfPassiveRobots = world->getWorld()->getUs().size() - 1;
    // Determine where behind our robots have to stand
    auto ballPosition = world->getWorld()->getBall();
    // If there is no ball, use the default division A penalty mark position
    double ballX = ballPosition.has_value() ? ballPosition.value()->position.x : PENALTY_MARK_THEM_X;
    double limitX = std::max(ballX, PENALTY_MARK_THEM_X) + Constants::PENALTY_DISTANCE_BEHIND_BALL();

    // First, figure out at what interval the robots will stand on a horizontal line
    double horizontalRange = std::fabs(field.playArea.right() - limitX);
    double horizontalHalfStep = horizontalRange / amountOfPassiveRobots;  // 5 robots for stepSize, divided by 2 for half stepSize

    // Then, figure out vertical stepSize
    double verticalRange = std::fabs(field.rightDefenseArea.bottom() - field.playArea.bottom());
    double verticalHalfStep = verticalRange / (2.0 * 2.0);  // 2 rows, divided by 2 for half stepSize

    double startX = field.playArea.right() - horizontalHalfStep;
    double bottomY = field.playArea.bottom() + verticalHalfStep;
    double topY = bottomY + 2 * verticalHalfStep;

    const std::string formationPrefix = "formation_";

    /// Bottom row of robots
    for (int i = 0; i < amountOfPassiveRobots / 2; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX - i * 2 * horizontalHalfStep, bottomY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.leftGoalArea.rightLine().center() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }

    /// Top row of robots
    for (int i = amountOfPassiveRobots / 2; i < amountOfPassiveRobots; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX - (i - amountOfPassiveRobots / 2) * 2 * horizontalHalfStep, topY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.leftGoalArea.rightLine().center() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }
}

const char* PenaltyThemPrepare::getName() const { return "Penalty Them Prepare"; }

}  // namespace rtt::ai::stp::play
