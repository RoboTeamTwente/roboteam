#include "stp/plays/referee_specific/PenaltyUsPrepare.h"

#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

// The x position on which we take the penalty
constexpr double PENALTY_MARK_US_X = -2.0;

PenaltyUsPrepare::PenaltyUsPrepare() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::PenaltyUsPrepareGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::Formation>("kicker_formation"),
        std::make_unique<role::Formation>("formation_0"),
        std::make_unique<role::Formation>("formation_1"),
        std::make_unique<role::Formation>("formation_2"),
        std::make_unique<role::Formation>("formation_3"),
        // Additional roles if we play 11v11
        std::make_unique<role::Formation>("formation_4"),
        std::make_unique<role::Formation>("formation_5"),
        std::make_unique<role::Formation>("formation_6"),
        std::make_unique<role::Formation>("formation_7"),
        std::make_unique<role::Formation>("formation_8"),
    };
}

uint8_t PenaltyUsPrepare::score(const rtt::Field& field) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap PenaltyUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kicker_formation", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"formation_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"formation_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void PenaltyUsPrepare::calculateInfoForRoles() noexcept {
    // We need at least a keeper, and a kicker positioned behind the ball
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.leftGoalArea.rightLine().center()));
    stpInfos["kicker_formation"].setPositionToMoveTo(world->getWorld()->getBall()->get()->position - Vector2{0.25, 0.0});

    // During our penalty, all our robots should be behind the ball to not interfere.
    // Create a grid pattern of robots on our side of the field
    int amountOfPassiveRobots = world->getWorld()->getUs().size() - 2;
    // Determine where behind our robots have to stand
    auto ballPosition = world->getWorld()->getBall();
    // If there is no ball, use the default division A penalty mark position
    double ballX = ballPosition.has_value() ? ballPosition.value()->position.x : PENALTY_MARK_US_X;
    double limitX = std::min(ballX, PENALTY_MARK_US_X) - Constants::PENALTY_DISTANCE_BEHIND_BALL();

    // Then, figure out at what interval the robots will stand on a horizontal line
    double horizontalRange = std::fabs(field.playArea.left() - limitX);
    double horizontalHalfStep = horizontalRange / amountOfPassiveRobots;  // 5 robots for stepSize, divided by 2 for half stepSize

    // Lastly, figure out vertical stepSize
    double verticalRange = std::fabs(field.leftDefenseArea.bottom() - field.playArea.bottom());
    double verticalHalfStep = verticalRange / (2.0 * 2.0);  // 2 rows, divided by 2 for half stepSize

    double startX = field.playArea.left() + horizontalHalfStep;
    double bottomY = field.playArea.bottom() + verticalHalfStep;
    double topY = bottomY + 2 * verticalHalfStep;

    const std::string formationPrefix = "formation_";

    /// Bottom row of robots
    for (int i = 0; i < amountOfPassiveRobots / 2; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX + i * 2 * horizontalHalfStep, bottomY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.rightGoalArea.leftLine().center() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }

    /// Top row of robots
    for (int i = amountOfPassiveRobots / 2; i < amountOfPassiveRobots; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX + (i - amountOfPassiveRobots / 2) * 2 * horizontalHalfStep, topY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.rightGoalArea.leftLine().center() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }
}

const char* PenaltyUsPrepare::getName() const { return "Penalty Us Prepare"; }
}  // namespace rtt::ai::stp::play
