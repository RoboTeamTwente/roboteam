#include "stp/computations/PositionComputations.h"

#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Tube.h>
#include <stp/computations/InterceptionComputations.h>

#include <roboteam_utils/Field.hpp>

#include "roboteam_utils/Hungarian.h"
#include "stp/Play.hpp"
#include "stp/computations/ComputationManager.h"
#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "utilities/Constants.h"
#include "world/World.hpp"

namespace rtt::ai::stp {
int PositionComputations::amountOfWallers = 4;

gen::ScoredPosition PositionComputations::getPosition(std::optional<rtt::Vector2> currentPosition, const Grid &searchGrid, gen::ScoreProfile profile, const Field &field,
                                                      const world::World *world) {
    gen::ScoredPosition bestPosition;
    AvoidObjects avoidObj;
    avoidObj.shouldAvoidOurDefenseArea = true;
    avoidObj.shouldAvoidTheirDefenseArea = true;
    avoidObj.shouldAvoidOutOfField = true;
    (currentPosition.has_value()) ? bestPosition = PositionScoring::scorePosition(currentPosition.value(), profile, field, world, 2) : bestPosition = {{0, 0}, 0};
    for (const auto &nestedPoints : searchGrid.getPoints()) {
        for (const Vector2 &position : nestedPoints) {
            if (!FieldComputations::pointIsValidPosition(field, position, avoidObj)) continue;
            gen::ScoredPosition consideredPosition = PositionScoring::scorePosition(position, profile, field, world);
            if (consideredPosition.score > bestPosition.score) bestPosition = consideredPosition;
        }
    }
    return bestPosition;
}

void PositionComputations::setAmountOfWallers(const rtt::Field &field, rtt::world::World *world) noexcept {
    Vector2 ballPos;
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();

    if (currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT ||
        currentGameState == RefCommand::PREPARE_FORCED_START) {
        ballPos = GameStateManager::getRefereeDesignatedPosition();
    } else {
        ballPos = FieldComputations::projectPointInField(field, world->getWorld().value().getBall()->get()->position, constants::BALL_RADIUS);
    }
    auto lineToBottomPost = ballPos - field.leftGoalArea.bottomRight();
    auto lineToTopPost = ballPos - field.leftGoalArea.topRight();
    auto angleBetweenLines = lineToBottomPost.toAngle().shortestAngleDiff(lineToTopPost.toAngle());
    if (field.rightPlayArea.contains(ballPos, -0.1)) {
        PositionComputations::amountOfWallers = 2;
    } else if (field.leftPlayArea.contains(ballPos)) {
        if (angleBetweenLines > 0.25)
            PositionComputations::amountOfWallers = 4;
        else if (angleBetweenLines > 0.2)
            PositionComputations::amountOfWallers = std::clamp(PositionComputations::amountOfWallers, 4, 3);
        else if (angleBetweenLines > 0.13)
            PositionComputations::amountOfWallers = 3;
        else if (angleBetweenLines > 0.1)
            PositionComputations::amountOfWallers = std::clamp(PositionComputations::amountOfWallers, 3, 2);
        else
            PositionComputations::amountOfWallers = 2;
    }
}

Vector2 PositionComputations::getWallPosition(int index, int amountDefenders, const rtt::Field &field, rtt::world::World *world) {
    if (ComputationManager::calculatedWallPositions.empty()) {
        ComputationManager::calculatedWallPositions = determineWallPositions(field, world, amountDefenders);
    }

    return ComputationManager::calculatedWallPositions[index];
}

std::vector<Vector2> PositionComputations::determineWallPositions(const rtt::Field &field, const rtt::world::World *world, int amountDefenders) {
    // Return an empty vector if there are no defenders
    if (amountDefenders <= 0) {
        return {};
    }

    // Constants for positioning the defenders
    const double radius = constants::ROBOT_RADIUS;
    const double spacingRobots = radius * 0.5;
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();
    auto ball = world->getWorld().value().getBall()->get();
    const LineSegment ballTrajectory(ball->position, ball->expectedEndPosition);
    Vector2 ballPos = ball->position;

    // Calculate the position of the ball, projected onto the field
    if (currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT ||
        currentGameState == RefCommand::PREPARE_FORCED_START) {
        // If the ball has to be placed somewhere on the field, assume the ball is already there
        ballPos = GameStateManager::getRefereeDesignatedPosition();
    } else if (field.leftDefenseArea.contains(ballPos)) {
        // If the ball is in our defense area, project it out of it
        ballPos = FieldComputations::projectPointOutOfDefenseArea(field, ballPos, true, false);
    } else if ((world->getWorld().value().getBall()->get()->velocity).length() > constants::BALL_GOT_SHOT_LIMIT &&
               InterceptionComputations::calculateTheirBallInterception(world, ballTrajectory).has_value()) {
        ballPos = *InterceptionComputations::calculateTheirBallInterception(world, ballTrajectory);
    } else {
        // Project the ball into the field and use that location
        ballPos = FieldComputations::projectPointInField(field, ballPos, constants::BALL_RADIUS);
    }
    // Dynamic distance, the further the ball is from our goal, the further forwards our wall stands.
    double theirDistanceToGoal = std::clamp(((ballPos - field.leftGoalArea.rightLine().center()).length() - 4) / 2, -constants::ROBOT_RADIUS * 1.9, field.playArea.width() / 10) +
                                 2 * constants::ROBOT_RADIUS;
    auto extraLength = (ballPos - field.leftGoalArea.rightLine().center()).stretchToLength(theirDistanceToGoal);

    std::vector<Vector2> positions = {};
    Vector2 projectedPosition;

    // Find the intersection of the ball-to-goal line with the border of the defense area
    LineSegment ball2GoalLine = LineSegment(ballPos, field.leftGoalArea.rightLine().center());
    std::vector<Vector2> lineBorderIntersects =
        FieldComputations::getDefenseArea(field, true, std::get<1>(FieldComputations::getDefenseAreaMargin()), 0).intersections(ball2GoalLine);
    // If the ball is in our defense area, project it outside, otherwise use the intersection with our defense area
    std::sort(lineBorderIntersects.begin(), lineBorderIntersects.end(), [](Vector2 a, Vector2 b) { return a.x > b.x; });
    projectedPosition = lineBorderIntersects.front() + extraLength;

    // Initialize the wallLine
    LineSegment wallLine;

    // Define the defense areas
    auto defenseAreaOur = FieldComputations::getDefenseArea(field, true, 0, 0);
    auto defenseAreaTheir = FieldComputations::getDefenseArea(field, false, 0, 0);

    // Determine the wall line based on the ball's position

    if (ballPos.y < field.leftDefenseArea.top() && ballPos.y > field.leftDefenseArea.bottom()) {
        // Case when it is to the right of our defense area
        double lineX = field.leftDefenseArea.right() + extraLength.x;
        double lineYTop = field.playArea.top() + extraLength.y;
        double lineYBottom = field.playArea.bottom() + extraLength.y;
        wallLine = LineSegment({lineX, lineYBottom}, {lineX, lineYTop});
    } else if (ballPos.x < field.leftDefenseArea.right()) {
        // Case when the projected position is below or above our defense area
        if (ballPos.y < 0) {
            wallLine = LineSegment(defenseAreaOur[0], defenseAreaTheir[0]);
            wallLine.move({extraLength.x, extraLength.y});
        } else {
            wallLine = LineSegment(defenseAreaOur[3], defenseAreaTheir[3]);
            wallLine.move({extraLength.x, extraLength.y});
        }
    } else {
        // We put the wall line perpendicular to the ball-goal line
        wallLine = LineSegment(ballPos, field.leftGoalArea.rightLine().center());
        wallLine.rotate(M_PI / 2, projectedPosition);

        // And resize it to make sure enough robots can fit on it
        double newLength = 2 * std::max(field.playArea.width(), field.playArea.height());
        wallLine.resize(newLength);

        // Limit this resizing to the edges of the field (we don't want to place robots outside of the field)
        auto oneHalf = LineSegment(projectedPosition, wallLine.start);
        auto intersectionOne = FieldComputations::lineIntersectionWithField(field, oneHalf.start, oneHalf.end, 0.0);
        if (intersectionOne.has_value()) {
            // Then limit this side of the wall line until this intersection
            wallLine.start = intersectionOne.value();
        }

        auto otherHalf = LineSegment(projectedPosition, wallLine.end);
        auto intersectionOther = FieldComputations::lineIntersectionWithField(field, otherHalf.start, otherHalf.end, 0.0);
        if (intersectionOther.has_value()) {
            // Then limit this side of the wall line until this intersection
            wallLine.end = intersectionOther.value();
        }
    }

    size_t defendersCount = static_cast<size_t>(amountDefenders);
    int i = 1;

    // If the number of defenders is even, start just outside of the projected position to make sure the projected position is the middle of the wall
    if (amountDefenders % 2 == 0) {
        int j = 1;
        while (positions.size() < defendersCount) {
            double circleRadius = (i - 0.5) * spacingRobots + j * radius;
            Circle circle = Circle(projectedPosition, circleRadius);
            std::vector<Vector2> intersects = circle.intersects(wallLine);
            positions.insert(positions.end(), intersects.begin(), intersects.end());
            j += 2;
            i += 1;
        }
    }
    // If the number of defenders is odd, start positioning from the projected position
    else {
        positions.push_back(projectedPosition);
        while (positions.size() < defendersCount) {
            double circleRadius = i * 2 * radius + spacingRobots * i;
            Circle circle = Circle(projectedPosition, circleRadius);
            std::vector<Vector2> intersects = circle.intersects(wallLine);
            positions.insert(positions.end(), intersects.begin(), intersects.end());
            i += 1;
        }
    }

    // Sort the positions to prevent robots from changing their position within the wall
    if (ballPos.x < field.leftDefenseArea.right()) {
        // If the ball is in the lower half of the field, sort by ascending x. Otherwise, sort by descending x.
        auto compareX = (ballPos.y < 0) ? [](Vector2 a, Vector2 b) { return a.x < b.x; } : [](Vector2 a, Vector2 b) { return a.x > b.x; };
        std::sort(std::begin(positions), std::end(positions), compareX);
    } else {
        // If the ball is in the right half of the field, sort by ascending y.
        std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.y < b.y; });
    }
    return positions;
}

Vector2 PositionComputations::calculateAvoidRobotsPosition(Vector2 targetPosition, const world::World *world, int robotId, const AvoidObjects &avoidObj, const Field &field) {
    std::vector<Vector2> pointsToAvoid = {};
    if (avoidObj.shouldAvoidOurRobots) {
        for (auto &robot : world->getWorld()->getUs()) {
            if (robot->getId() != robotId) {
                pointsToAvoid.push_back(robot->getPos());
            }
        }
    }
    if (avoidObj.shouldAvoidTheirRobots) {
        for (auto &robot : world->getWorld()->getThem()) {
            pointsToAvoid.push_back(robot->getPos());
        }
    }
    // We use robot radius instead of 2 * robot radius to make sure we are not overly cautious
    if (std::all_of(pointsToAvoid.begin(), pointsToAvoid.end(), [&](const Vector2 &avoidPoint) { return avoidPoint.dist(targetPosition) >= constants::ROBOT_RADIUS; })) {
        return targetPosition;
    }

    for (int distanceSteps = 0; distanceSteps < 5; ++distanceSteps) {
        auto distance = 4 * constants::ROBOT_RADIUS + distanceSteps * constants::ROBOT_RADIUS * 2;
        auto possiblePoints = Grid(targetPosition.x - distance / 2.0, targetPosition.y - distance / 2.0, distance, distance, 3, 3).getPoints();
        for (auto &pointVector : possiblePoints) {
            for (auto &point : pointVector) {
                if (FieldComputations::pointIsValidPosition(field, point, avoidObj) &&
                    std::all_of(pointsToAvoid.begin(), pointsToAvoid.end(), [&](const Vector2 &avoidPoint) { return avoidPoint.dist(point) >= constants::ROBOT_RADIUS; })) {
                    return point;
                }
            }
        }
    }
    RTT_WARNING("Could not find good position to avoid robots for robot with id: " + std::to_string(robotId));
    return targetPosition;
}

Vector2 PositionComputations::calculateAvoidBallPosition(Vector2 targetPosition, Vector2 ballPosition, const Field &field, const AvoidObjects &avoidObj) {
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();

    std::unique_ptr<Shape> avoidShape;

    if (currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT ||
        currentGameState == RefCommand::PREPARE_FORCED_START) {
        avoidShape =
            std::make_unique<Tube>(Tube(ballPosition, GameStateManager::getRefereeDesignatedPosition(), constants::AVOID_BALL_DISTANCE + constants::GO_TO_POS_ERROR_MARGIN));
    } else {
        avoidShape = std::make_unique<Circle>(Circle(ballPosition, constants::AVOID_BALL_DISTANCE + constants::GO_TO_POS_ERROR_MARGIN));
    }

    if (avoidShape->contains(targetPosition)) {
        auto projectedPos = avoidShape->project(targetPosition);
        if (FieldComputations::pointIsValidPosition(field, projectedPos, avoidObj))
            targetPosition = projectedPos;
        else {
            targetPosition = calculatePositionOutsideOfShape(targetPosition, field, avoidShape, avoidObj);
        }
    }
    return targetPosition;
}

Vector2 PositionComputations::calculatePositionOutsideOfShape(Vector2 targetPosition, const rtt::Field &field, const std::unique_ptr<Shape> &avoidShape,
                                                              const AvoidObjects &avoidObj) {
    for (int distanceSteps = 0; distanceSteps < 5; ++distanceSteps) {
        auto distance = 2 * constants::AVOID_BALL_DISTANCE + distanceSteps * constants::AVOID_BALL_DISTANCE;
        auto possiblePoints = Grid(targetPosition.x - distance / 2.0, targetPosition.y - distance / 2.0, distance, distance, 3, 3).getPoints();
        for (auto &pointVector : possiblePoints) {
            for (auto &point : pointVector) {
                if (FieldComputations::pointIsValidPosition(field, point, avoidObj) && !avoidShape->contains(point)) {
                    return point;
                }
            }
        }
    }
    RTT_WARNING("Could not find good position to avoid ball");
    return targetPosition;
}

void PositionComputations::calculateInfoForHarasser(std::unordered_map<std::string, StpInfo> &stpInfos, std::array<std::unique_ptr<Role>, constants::MAX_ROBOT_COUNT> *roles,
                                                    const Field &field, world::World *world, Vector2 interceptionLocation) noexcept {
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToPoint(world->getWorld()->getBall()->get()->position, world::them);
    auto harasserAngle = stpInfos["harasser"].getYaw();
    // If there is no enemy or we don't have a harasser yet, estimate the position to move to
    if (!stpInfos["harasser"].getRobot() || !enemyClosestToBall) {
        stpInfos["harasser"].setPositionToMoveTo(interceptionLocation);
        stpInfos["harasser"].setYaw((world->getWorld()->getBall()->get()->position - interceptionLocation).angle());
        return;
    }
    auto enemyAngle = enemyClosestToBall->get()->getYaw();
    // If enemy is not facing our goal AND does have the ball, stand between the enemy and our goal
    if (enemyClosestToBall->get()->hasBall() && enemyAngle.shortestAngleDiff(harasserAngle) < M_PI / 1.5) {
        auto enemyPos = enemyClosestToBall->get()->getPos();
        auto targetPos = enemyPos + (field.leftGoalArea.leftLine().center() - enemyPos).stretchToLength(constants::ROBOT_RADIUS * 4 + constants::GO_TO_POS_ERROR_MARGIN);
        stpInfos["harasser"].setPositionToMoveTo(targetPos);
        stpInfos["harasser"].setYaw((world->getWorld()->getBall()->get()->position - targetPos).angle());
    } else {
        auto harasser = std::find_if(roles->begin(), roles->end(), [](const std::unique_ptr<Role> &role) { return role != nullptr && role->getName() == "harasser"; });
        if (harasser != roles->end() && !harasser->get()->finished() && strcmp(harasser->get()->getCurrentTactic()->getName(), "Formation") == 0) {
            auto enemyPos = enemyClosestToBall->get()->getPos();
            auto targetPos = enemyPos + (world->getWorld()->getBall()->get()->position - enemyPos).stretchToLength(constants::ROBOT_RADIUS * 3);
            if (enemyClosestToBall->get()->hasBall() && ((stpInfos["harasser"].getRobot()->get()->getPos() - targetPos).length() > 1.5 * constants::GO_TO_POS_ERROR_MARGIN)) {
                stpInfos["harasser"].setPositionToMoveTo(targetPos);
                stpInfos["harasser"].setYaw((world->getWorld()->getBall()->get()->position - targetPos).angle());
            } else {
                harasser->get()->forceNextTactic();
            }
        }
    }
}

void PositionComputations::calculateInfoForDefendersAndWallers(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                               std::array<std::unique_ptr<Role>, constants::MAX_ROBOT_COUNT> &roles, const Field &field, world::World *world,
                                                               bool mustStayOnOurSide) noexcept {
    // List of all active defender and waller names
    auto defenderNames = std::vector<std::string>{};
    auto wallerNames = std::vector<std::string>{};
    auto additionalWallerNames = std::vector<std::string>{};
    PositionComputations::setAmountOfWallers(field, world);
    for (size_t i = 0; i < world->getWorld()->getUs().size(); i++) {
        if (roles[i]->getName().find("waller") != std::string::npos && static_cast<int>(wallerNames.size()) < PositionComputations::amountOfWallers) {
            wallerNames.emplace_back(roles[i]->getName());
        } else if (roles[i]->getName().find("waller") != std::string::npos) {
            defenderNames.emplace_back(roles[i]->getName());
        } else if (roles[i]->getName().find("defender") != std::string::npos) {
            defenderNames.emplace_back(roles[i]->getName());
        }
    }
    auto activeDefenderNames = std::vector<std::string>{};
    for (auto name : defenderNames) {
        if (stpInfos[name].getRobot().has_value()) activeDefenderNames.emplace_back(name);
    }
    // Calculate the n closest enemies
    auto enemyRobots = world->getWorld()->getThem();
    std::optional<rtt::world::view::RobotView> enemyToIgnore;
    if (GameStateManager::getCurrentGameState().getCommandId() == RefCommand::BALL_PLACEMENT_THEM) {
        enemyToIgnore = world->getWorld()->getRobotClosestToPoint(GameStateManager::getRefereeDesignatedPosition(), world::them);
    } else {
        enemyToIgnore = world->getWorld()->getRobotClosestToBall(world::them);
    }
    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyToIgnore && enemyRobot->getId() == enemyToIgnore.value()->getId(); });
    std::map<double, EnemyInfo> enemyMap;
    std::vector<Vector2> enemies;
    for (auto enemy : enemyRobots) {
        if (enemy->hasBall()) continue;
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        if (std::find(ComputationManager::calculatedEnemyMapIds.begin(), ComputationManager::calculatedEnemyMapIds.end(), enemy->getId()) !=
            ComputationManager::calculatedEnemyMapIds.end()) {
            score *= constants::ENEMY_ALREADY_ASSIGNED_MULTIPLIER;
        }
        EnemyInfo info = {enemy->getPos(), enemy->getVel(), enemy->getId()};
        enemyMap.insert({score, info});
    }
    ComputationManager::calculatedEnemyMapIds.clear();
    // If defenders do not have a position yet, don't do hungarian algorithm
    if (activeDefenderNames.empty()) {
        size_t loopSize = std::min(defenderNames.size(), enemyMap.size());
        for (size_t i = 0; i < loopSize; i++) {
            Vector2 defendPostion = enemyMap.begin()->second.position;
            if (mustStayOnOurSide && defendPostion.x > 0) {
                defendPostion.x = 0.0;
            }
            stpInfos[defenderNames[i]].setPositionToDefend(defendPostion);
            ComputationManager::calculatedEnemyMapIds.emplace_back(enemyMap.begin()->second.id);
            enemyMap.erase(enemyMap.begin());
        }
        for (size_t i = loopSize; i < defenderNames.size(); i++) {
            additionalWallerNames.emplace_back(defenderNames[i]);
        }
    } else {
        // Calculate the distance between the defenders and their enemies
        std::vector<std::vector<double>> cost_matrix;
        cost_matrix.resize(activeDefenderNames.size());
        size_t row_length = std::min(activeDefenderNames.size(), enemyMap.size());
        for (size_t i = 0; i < row_length; i++) {
            enemies.emplace_back((mustStayOnOurSide && (enemyMap.begin()->second.position.x > 0.0)) ? Vector2{0.0, enemyMap.begin()->second.position.y}
                                                                                                    : enemyMap.begin()->second.position);
            ComputationManager::calculatedEnemyMapIds.emplace_back(enemyMap.begin()->second.id);
            enemyMap.erase(enemyMap.begin());
        }
        for (size_t i = 0; i < activeDefenderNames.size(); i++) {
            cost_matrix[i].resize(row_length);
            for (size_t j = 0; j < row_length; j++) {
                cost_matrix[i][j] = stpInfos[activeDefenderNames[i]].getRobot()->get()->getPos().dist(enemies[j]);
            }
        }
        // Calculate the optimal assignment of enemies to pass_defenders using the hungarian algorithm and set the position to defend for each
        // active pass defender
        std::vector<int> assignments;
        rtt::Hungarian::Solve(cost_matrix, assignments);
        for (size_t i = 0; i < activeDefenderNames.size(); i++) {
            // If assignments is -1, it means the pass defender does not get an enemy assigned to it, because there are more pass defenders than enemies
            if (assignments[i] == -1) {
                additionalWallerNames.emplace_back(activeDefenderNames[i]);
            } else {
                stpInfos[activeDefenderNames[i]].setPositionToDefend(enemies[assignments[i]]);
            }
        }
    }
    for (size_t i = 0; i < wallerNames.size() + additionalWallerNames.size(); ++i) {
        // For each waller, stand in the right wall position and look at the ball
        auto positionToMoveTo = PositionComputations::getWallPosition(i, wallerNames.size() + additionalWallerNames.size(), field, world);
        std::string currentWallerName;
        if (i < additionalWallerNames.size() / 2) {
            currentWallerName = additionalWallerNames[i];
        } else if (i < wallerNames.size() + additionalWallerNames.size() / 2) {
            currentWallerName = wallerNames[i - additionalWallerNames.size() / 2];
        } else {
            currentWallerName = additionalWallerNames[i - wallerNames.size()];
        }
        auto &wallerStpInfo = stpInfos[currentWallerName];

        wallerStpInfo.setPositionToMoveTo(positionToMoveTo);
        wallerStpInfo.setYaw((world->getWorld()->getBall()->get()->position - field.leftGoalArea.rightLine().center()).angle());

        // If the waller is close to its target, ignore collisions
        constexpr double IGNORE_COLLISIONS_DISTANCE = 0.4;
        if (wallerStpInfo.getRobot() && (wallerStpInfo.getRobot()->get()->getPos() - positionToMoveTo).length() < IGNORE_COLLISIONS_DISTANCE) {
            wallerStpInfo.setShouldAvoidTheirRobots(false);
        }
    }
}

void PositionComputations::calculateInfoForAttackers(std::unordered_map<std::string, StpInfo> &stpInfos, std::array<std::unique_ptr<Role>, constants::MAX_ROBOT_COUNT> &roles,
                                                     const Field &field, world::World *world) noexcept {
    // List of all active attackers
    auto attackerNames = std::vector<std::string>{};
    for (size_t i = 0; i < world->getWorld()->getUs().size(); i++) {
        if (roles[i]->getName().find("attacker") != std::string::npos) {
            attackerNames.emplace_back(roles[i]->getName());
        }
    }
    if (attackerNames.size() == 0)
        ;  // Do nothing
    else if (attackerNames.size() == 1) {
        stpInfos["attacker_0"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleRightGrid, gen::AttackingPass, field, world));
    } else if (attackerNames.size() == 2) {
        stpInfos["attacker_0"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topRightGrid, gen::AttackingPass, field, world));
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomRightGrid, gen::AttackingPass, field, world));
    } else if (attackerNames.size() >= 3) {
        stpInfos["attacker_0"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topRightGrid, gen::SafePass, field, world));
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleRightGrid, gen::AttackingPass, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomRightGrid, gen::SafePass, field, world));
    }
    if (attackerNames.size() == 4) {
        stpInfos["attacker_3"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleMidGrid, gen::AttackingPass, field, world));
    } else if (attackerNames.size() == 5) {
        stpInfos["attacker_3"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::AttackingPass, field, world));
        stpInfos["attacker_4"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::AttackingPass, field, world));
    } else if (attackerNames.size() >= 6) {
        stpInfos["attacker_3"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::AttackingPass, field, world));
        stpInfos["attacker_4"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleMidGrid, gen::SafePass, field, world));
        stpInfos["attacker_5"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::AttackingPass, field, world));
    }
}

void PositionComputations::calculateInfoForFormation(std::unordered_map<std::string, StpInfo> &stpInfos, std::array<std::unique_ptr<Role>, constants::MAX_ROBOT_COUNT> &roles,
                                                     const Field &field, world::World *world) noexcept {
    auto formationBackNames = std::vector<std::string>{};
    auto formationMidNames = std::vector<std::string>{};
    auto formationFrontNames = std::vector<std::string>{};
    for (size_t i = 0; i < world->getWorld()->getUs().size(); i++) {
        if (roles[i]->getName().find("formation_back") != std::string::npos) {
            formationBackNames.emplace_back(roles[i]->getName());
        } else if (roles[i]->getName().find("formation_mid") != std::string::npos) {
            formationMidNames.emplace_back(roles[i]->getName());
        } else if (roles[i]->getName().find("formation_front") != std::string::npos) {
            formationFrontNames.emplace_back(roles[i]->getName());
        }
    }

    auto width = field.playArea.width();
    auto height = field.playArea.height();

    for (size_t i = 0; i < formationBackNames.size(); i++) {
        stpInfos[formationBackNames[i]].setPositionToMoveTo(Vector2{-width / 3.5, -height / 2 + height / (formationBackNames.size() + 1) * (i + 1)});
    }
    for (size_t i = 0; i < formationMidNames.size(); i++) {
        stpInfos[formationMidNames[i]].setPositionToMoveTo(Vector2{-width / 10, -height / 2 + height / (formationMidNames.size() + 1) * (i + 1)});
    }
    for (size_t i = 0; i < formationFrontNames.size(); i++) {
        stpInfos[formationFrontNames[i]].setPositionToMoveTo(Vector2{width / 8, -height / 2 + height / (formationFrontNames.size() + 1) * (i + 1)});
    }
}

void PositionComputations::calculateInfoForFormationOurSide(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                            std::array<std::unique_ptr<Role>, constants::MAX_ROBOT_COUNT> &roles, const Field &field,
                                                            world::World *world) noexcept {
    auto formationBackNames = std::vector<std::string>{};
    auto formationMidNames = std::vector<std::string>{};
    auto formationFrontNames = std::vector<std::string>{};
    for (size_t i = 0; i < world->getWorld()->getUs().size(); i++) {
        if (roles[i]->getName().find("formation_back") != std::string::npos) {
            formationBackNames.emplace_back(roles[i]->getName());
        } else if (roles[i]->getName().find("formation_mid") != std::string::npos) {
            formationMidNames.emplace_back(roles[i]->getName());
        } else if (roles[i]->getName().find("formation_front") != std::string::npos) {
            formationFrontNames.emplace_back(roles[i]->getName());
        }
    }

    auto width = field.playArea.width();
    auto height = field.playArea.height();

    for (size_t i = 0; i < formationBackNames.size(); i++) {
        stpInfos[formationBackNames[i]].setPositionToMoveTo(Vector2{-width / 3, -height / 8 + height / (formationBackNames.size() + 1) * (i + 1) / 4});
    }
    for (size_t i = 0; i < formationMidNames.size(); i++) {
        stpInfos[formationMidNames[i]].setPositionToMoveTo(Vector2{-width / 5, -height / 2 + height / (formationMidNames.size() + 1) * (i + 1)});
    }
    for (size_t i = 0; i < formationFrontNames.size(); i++) {
        double y = -height / 2 + height / (formationFrontNames.size() + 1) * (i + 1);
        stpInfos[formationFrontNames[i]].setPositionToMoveTo(Vector2{-constants::ROBOT_RADIUS * 1.5, y});
    }
}

void PositionComputations::recalculateInfoForNonPassers(std::unordered_map<std::string, StpInfo> &stpInfos, const Field &field, world::World *world,
                                                        Vector2 receiverLocation) noexcept {
    auto ballPosition = world->getWorld()->getBall()->get()->position;
    // Make a list of all robots that are not the passer, receiver or keeper, which need to make sure they are not in the way of the pass
    auto toBeCheckedRobots = std::vector<std::string>{};
    for (auto &role : stpInfos) {
        if (role.second.getRobot().has_value()) {
            auto robotName = role.first;
            if (robotName != "keeper" && robotName != "passer" && robotName != "receiver" && robotName != "striker" && robotName != "free_kick_taker" &&
                robotName.find("waller") == std::string::npos) {
                toBeCheckedRobots.emplace_back(role.first);
            }
        }
    }
    // Make a tube around the pass trajectory, and make sure all robots outside of this tube
    std::unique_ptr<Shape> avoidShape = std::make_unique<Tube>(Tube(ballPosition, receiverLocation, constants::DISTANCE_TO_PASS_TRAJECTORY));
    for (auto &robot : toBeCheckedRobots) {
        stpInfos[robot].setShouldAvoidBall(true);
        auto robotPositionToMoveTo = stpInfos[robot].getPositionToMoveTo();
        if (robotPositionToMoveTo == std::nullopt || !robotPositionToMoveTo.has_value()) {
            continue;
        }
        if (!avoidShape->contains(robotPositionToMoveTo.value())) {
            continue;
        }
        auto newRobotPositionToMoveTo = calculatePositionOutsideOfShape(robotPositionToMoveTo.value(), field, avoidShape, stpInfos[robot].getObjectsToAvoid());
        stpInfos[robot].setPositionToMoveTo(newRobotPositionToMoveTo);
    }
}

void PositionComputations::calculateInfoForAvoidBallHarasser(std::unordered_map<std::string, StpInfo> &stpInfos, world::World *world) noexcept {
    if (!world->getWorld()->getBall()) return;
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto goalToBall = ballPos - world->getField().value().leftGoalArea.rightLine().center();
    stpInfos["harasser"].setPositionToMoveTo(ballPos - goalToBall.stretchToLength(constants::AVOID_BALL_DISTANCE));
}

}  // namespace rtt::ai::stp