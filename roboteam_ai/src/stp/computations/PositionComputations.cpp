//
// Created by maxl on 09-02-21.
//

#include "stp/computations/PositionComputations.h"

#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Tube.h>

#include <roboteam_utils/Field.hpp>
#include "roboteam_utils/Hungarian.h"
#include "stp/computations/ComputationManager.h"
#include "stp/computations/PositionScoring.h"
#include "world/World.hpp"

namespace rtt::ai::stp {

gen::ScoredPosition PositionComputations::getPosition(std::optional<rtt::Vector2> currentPosition, const Grid &searchGrid, gen::ScoreProfile profile, const Field &field,
                                                      const world::World *world) {
    gen::ScoredPosition bestPosition;
    (currentPosition.has_value()) ? bestPosition = PositionScoring::scorePosition(currentPosition.value(), profile, field, world, 2) : bestPosition = {{0, 0}, 0};
    for (const auto &nestedPoints : searchGrid.getPoints()) {
        for (const Vector2 &position : nestedPoints) {
            if (!FieldComputations::pointIsValidPosition(field, position)) continue;
            gen::ScoredPosition consideredPosition = PositionScoring::scorePosition(position, profile, field, world);
            if (consideredPosition.score > bestPosition.score) bestPosition = consideredPosition;
        }
    }
    return bestPosition;
}

Vector2 PositionComputations::getWallPosition(int index, int amountDefenders, const rtt::Field &field, rtt::world::World *world) {
    if (ComputationManager::calculatedWallPositions.empty()) {
        ComputationManager::calculatedWallPositions = determineWallPositions(field, world, amountDefenders);
    }

    return ComputationManager::calculatedWallPositions[index];
}

std::vector<Vector2> PositionComputations::determineWallPositions(const rtt::Field &field, const rtt::world::World *world, int amountDefenders) {
    if (amountDefenders <= 0) return {};  // we need at least 1 defender to be able to compute a wall

    double radius = control_constants::ROBOT_RADIUS;
    double spacingRobots = radius * 0.5;
    double spaceBetweenDefenseArea = 2 * radius;

    Vector2 ballPos = FieldComputations::projectPointInField(field, world->getWorld().value().getBall()->get()->position);

    std::vector<Vector2> positions = {};

    Vector2 projectedPosition;
    std::vector<Vector2> lineBorderIntersects = {};

    /// Get defense area border geometry
    std::vector<LineSegment> defenseAreaBorder = FieldComputations::getDefenseArea(field, true, spaceBetweenDefenseArea, 0).getBoundary();

    /// Find intersect of ball to goal on the border of the defense area
    LineSegment ball2GoalLine = LineSegment(ballPos, field.leftGoalArea.rightLine().center());

    lineBorderIntersects = FieldComputations::getDefenseArea(field, true, spaceBetweenDefenseArea, 0).intersections(ball2GoalLine);

    if (!lineBorderIntersects.empty()) {
        std::sort(std::begin(lineBorderIntersects), std::end(lineBorderIntersects), [](Vector2 a, Vector2 b) { return a.x > b.x; });
        projectedPosition = lineBorderIntersects.front();  // Always use the first one
    } else {
        projectedPosition = Vector2{field.leftGoalArea.rightLine().center().x, field.leftDefenseArea.bottom()};
    }

    LineSegment wallLine;

    // TODO: The wall line used to be as long as the line of the defense area it belonged to.
    // This meant that later, when we intersect it with a circle to decide the positions of the robots,
    // there is a possibility that the circle becomes bigger than the linesegment, resulting in an infinite loop.
    // A quick fix for this is extending the line segments, but in my eyes, it is a beun fix, because there is
    // still no guarantee it is big enough
    if (ballPos.x < field.leftDefenseArea.right()) {
        // case when the projected position is below penalty line
        if (ballPos.y < 0) {
            wallLine = LineSegment(Vector2{FieldComputations::getDefenseArea(field, true, 0, 0)[0].x, FieldComputations::getDefenseArea(field, true, 0, 0)[0].y},
                                   Vector2{FieldComputations::getDefenseArea(field, false, 0, 0)[0].x, FieldComputations::getDefenseArea(field, false, 0, 0)[0].y});
            //            wallLine = LineSegment(field.leftDefenseArea.bottomLeft(), field.rightDefenseArea.bottomRight());
            wallLine.move({0, -spaceBetweenDefenseArea});
        } else {
            wallLine = LineSegment(Vector2{FieldComputations::getDefenseArea(field, true, 0, 0)[3].x, FieldComputations::getDefenseArea(field, true, 0, 0)[3].y},
                                   Vector2{FieldComputations::getDefenseArea(field, false, 0, 0)[3].x, FieldComputations::getDefenseArea(field, false, 0, 0)[3].y});
            //            wallLine = LineSegment(field.leftDefenseArea.topLeft(), field.rightDefenseArea.topRight());
            wallLine.move({0, spaceBetweenDefenseArea});
        }
        // case when it is above the penalty line no further away than side lines of the defense area
    } else if (ballPos.y < field.leftDefenseArea.top() && ballPos.y > field.leftDefenseArea.bottom()) {
        double lineX = field.leftDefenseArea.right() + spaceBetweenDefenseArea;
        double lineYTop = field.playArea.top();
        double lineYBottom = field.playArea.bottom();
        wallLine = LineSegment({lineX, lineYBottom}, {lineX, lineYTop});
    } else {
        // We put the wall line perpendicular to the ball-goal line
        wallLine = LineSegment(ballPos, field.leftGoalArea.rightLine().center());
        wallLine.rotate(M_PI / 2, projectedPosition);

        // And resize it to make sure enough robots can fit on it
        double newLength = 2 * std::max(field.playArea.width(), field.playArea.height());
        wallLine.resize(newLength);

        // But limit this resizing to the edges of the field (we dont want to place robots outside of the field
        auto oneHalf = LineSegment(wallLine.center(), wallLine.start);
        auto intersectionOne = FieldComputations::lineIntersectionWithField(field, oneHalf.start, oneHalf.end, 0.0);
        if (intersectionOne.has_value()) {
            // Then limit this side of the wall line until this intersection
            wallLine.start = intersectionOne.value();
        }

        auto otherHalf = LineSegment(wallLine.center(), wallLine.end);
        auto intersectionOther = FieldComputations::lineIntersectionWithField(field, otherHalf.start, otherHalf.end, 0.0);
        if (intersectionOther.has_value()) {
            // Then limit this side of the wall line until this intersection
            wallLine.end = intersectionOther.value();
        }
    }

    int i = 1;
    if (amountDefenders % 2 == 0) {
        int j = 1;
        while (positions.size() < static_cast<size_t>(amountDefenders)) {
            auto circle = Circle(projectedPosition, (i - 0.5) * spacingRobots + j * radius);
            std::vector<Vector2> intersects;
            intersects = circle.intersects(wallLine);
            for (auto intersect : intersects) {
                positions.push_back(intersect);
            }
            j = j + 2;
            i = i + 1;
        }
    } else {
        positions.push_back(projectedPosition);
        while (positions.size() < static_cast<size_t>(amountDefenders)) {
            auto circle = Circle(projectedPosition, i * 2 * radius + spacingRobots * i);
            std::vector<Vector2> intersects;
            intersects = circle.intersects(wallLine);
            for (auto intersect : intersects) {
                positions.push_back(intersect);
            }
            i = i + 1;
        }
    }

    // For the robots not to change the position withing the wall
    if (ballPos.x < field.leftDefenseArea.right()) {
        if (ballPos.y < 0) {
            std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.x < b.x; });
        } else {
            std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.x > b.x; });
        }
    } else {
        std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.y < b.y; });
    }

    return positions;
}
Vector2 PositionComputations::calculateAvoidBallPosition(Vector2 targetPosition, Vector2 ballPosition, const Field &field) {
    auto currentGameState = GameStateManager::getCurrentGameState().getStrategyName();

    std::unique_ptr<Shape> avoidShape;

    // During ball placement, we need to avoid the area between the ball and the target position by a certain margin
    if (currentGameState == "ball_placement_us" || currentGameState == "ball_placement_them") {
        avoidShape = std::make_unique<Tube>(Tube(ballPosition, GameStateManager::getRefereeDesignatedPosition(), control_constants::AVOID_BALL_DISTANCE));
    } else {
        // During stop gamestate, we need to avoid the area directly around the ball.
        avoidShape = std::make_unique<Circle>(Circle(ballPosition, control_constants::AVOID_BALL_DISTANCE));
    }

    if (avoidShape->contains(targetPosition)) {
        auto projectedPos = avoidShape->project(targetPosition);
        if (FieldComputations::pointIsValidPosition(field, projectedPos))
            targetPosition = projectedPos;
        else {
            targetPosition = calculatePositionOutsideOfShape(ballPosition, field, avoidShape);
        }
    }
    return targetPosition;
}

Vector2 PositionComputations::calculatePositionOutsideOfShape(Vector2 ballPos, const rtt::Field &field, const std::unique_ptr<Shape> &avoidShape) {
    Vector2 newTarget = ballPos;  // The new position to go to
    bool pointFound = false;
    for (int distanceSteps = 0; distanceSteps < 5; ++distanceSteps) {
        // Use a larger grid each iteration in case no valid point is found
        auto distance = 3 * control_constants::AVOID_BALL_DISTANCE + distanceSteps * control_constants::AVOID_BALL_DISTANCE / 2.0;
        auto possiblePoints = Grid(ballPos.x - distance / 2.0, ballPos.y - distance / 2.0, distance, distance, 3, 3).getPoints();
        double dist = 1e3;
        for (auto &pointVector : possiblePoints) {
            for (auto &point : pointVector) {
                if (FieldComputations::pointIsValidPosition(field, point) && !avoidShape->contains(point)) {
                    if (ballPos.dist(point) < dist) {
                        dist = ballPos.dist(point);
                        newTarget = point;
                        pointFound = true;
                    }
                }
            }
        }
        if (pointFound) break;  // As soon as a valid point is found, don't look at more points further away
    }
    if (newTarget == ballPos) RTT_WARNING("Could not find good position to avoid ball");
    return newTarget;
}

// De prullenbak in?
Vector2 PositionComputations::getBallBlockPosition(const Field &field, const world::World *world) {
    if (!world->getWorld()->getBall()) return {field.leftPenaltyPoint};  // If there is no ball, return a default value

    constexpr double distFromDefenceArea = 1.0;

    // If the ball is within this distFromDefence area, go to the ball
    if (FieldComputations::getDefenseArea(field, true, distFromDefenceArea, 0).contains(world->getWorld()->getBall()->get()->position)) {
        return world->getWorld()->getBall()->get()->position;
    }
    // If the ball is moving towards our defense area, stand on its trajectory
    auto ball = world->getWorld()->getBall()->get();
    if (ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT) {
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        auto interceptionPoint = FieldComputations::lineIntersectionWithDefenseArea(field, true, ballTrajectory.start, ballTrajectory.end, distFromDefenceArea, true);
        if (interceptionPoint) return *interceptionPoint;
    }

    // If there is an enemy close to the ball and it is looking towards our goal, stand on the line that the enemy is looking in
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(rtt::world::Team::them);
    if (enemyClosestToBall && enemyClosestToBall->get()->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
        auto start = enemyClosestToBall->get()->getPos();
        auto robotAngle = enemyClosestToBall->get()->getAngle();
        auto end = start + robotAngle.toVector2().stretchToLength(field.playArea.width());
        // auto intersection = LineSegment(field.leftGoalArea.bottomRight() - Vector2(0, 0.20), field.leftGoalArea.topRight() + Vector2(0, 0.20)).intersects({start, end});
        auto intersection = FieldComputations::lineIntersectionWithDefenseArea(field, true, start, end, distFromDefenceArea, true);
        if (intersection != nullptr) {
            return FieldComputations::projectPointToValidPositionOnLine(field, *intersection, start, end, AvoidObjects(), 0.0, distFromDefenceArea, distFromDefenceArea);
        }
    }

    // If there is no enemy about to shoot and the ball is not moving towards the goal, simply stand in between the ball and our goal center
    auto ballToGoalIntersection =
        FieldComputations::lineIntersectionWithDefenseArea(field, true, ball->position, field.leftGoalArea.rightLine().center(), distFromDefenceArea, true);
    if (ballToGoalIntersection) return *ballToGoalIntersection;

    // If there is no ball to goal intersection (this essentially means the ball is in our defense area), project that the ball position to a valid point
    return {FieldComputations::projectPointToValidPosition(field, ball->position, AvoidObjects{}, 0.0, distFromDefenceArea, distFromDefenceArea)};
}

void PositionComputations::calculateInfoForKeeper(std::unordered_map<std::string, StpInfo> &stpInfos, const Field &field, world::World *world) noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.leftGoalArea.rightLine().center());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
}

void PositionComputations::calculateInfoForHarasser(std::unordered_map<std::string, StpInfo> stpInfos,
                                                    std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> *roles, const Field &field,
                                                    world::World *world) noexcept {
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);
    // If there is no enemy or we don't have a harasser yet, estimate the position to move to
    if (!stpInfos["harasser"].getRobot() || !enemyClosestToBall) {
        stpInfos["harasser"].setPositionToMoveTo(world->getWorld()->getBall()->get()->position);
        return;
    }
    auto ballPos = world->getWorld()->getBall()->get()->position;
    auto enemyAngle = enemyClosestToBall->get()->getAngle();
    auto enemyToGoalAngle = (field.leftGoalArea.leftLine().center() - enemyClosestToBall->get()->getPos()).angle();

    // If enemy is more than 90 degrees away from our goal AND does have the ball, stand between the enemy and our goal
    if (enemyClosestToBall->get()->hasBall() && enemyAngle.shortestAngleDiff(enemyToGoalAngle) > M_PI / 2) {
        auto enemyPos = enemyClosestToBall->get()->getPos();
        auto targetPos = FieldComputations::projectPointToValidPositionOnLine(
            field, enemyPos - (field.leftGoalArea.leftLine().center() - enemyPos).stretchToLength(control_constants::ROBOT_RADIUS), enemyPos,
            enemyPos - (field.leftGoalArea.leftLine().center() - enemyPos).stretchToLength(10), AvoidObjects{}, 0.0, control_constants::ROBOT_RADIUS * 2, 0.0);
        stpInfos["harasser"].setPositionToMoveTo(targetPos);
        stpInfos["harasser"].setAngle((ballPos - targetPos).angle());
        // Maybe reset such that we go to formation tactic?
    } else {
        // Allow the harasser to get close to the enemy robot by not caring about collisions with enemy robots and go to getBall tactic
        stpInfos["harasser"].setShouldAvoidTheirRobots(false);
        auto harasser = std::find_if(roles->begin(), roles->end(), [](const std::unique_ptr<Role> &role) { return role != nullptr && role->getName() == "harasser"; });
        if (harasser != roles->end() && !harasser->get()->finished() && strcmp(harasser->get()->getCurrentTactic()->getName(), "Formation") == 0)
            harasser->get()->forceNextTactic();
    }
}

void PositionComputations::calculateInfoForDefenders(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                     std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles, const Field &field,
                                                     world::World *world) noexcept {
    // List of all active defender, such that we can defend the n closests enemies
    auto defenderNames = std::vector<std::string>{};
    for (int i = 0; i < world->getWorld()->getUs().size(); i++) {
        if (roles[i]->getName().find("defender") != std::string::npos) {
            defenderNames.emplace_back(roles[i]->getName());
        }
    }
    auto activeDefenderNames = std::vector<std::string>{};
    for (auto name : defenderNames) {
        if (stpInfos[name].getRobot().has_value()) activeDefenderNames.emplace_back(name);
    }
    // Calculate the n closest enemies
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);
    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });
    std::map<double, Vector2> enemyMap;
    std::vector<Vector2> enemies;
    for (auto enemy : enemyRobots) {
        if (enemy->hasBall()) continue;
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        enemyMap.insert({score, enemy->getPos()});
    }
    // If defenders do not have a position yet, don't do hungarian algorithm
    if (activeDefenderNames.empty()) {
        auto loopSize = std::min(defenderNames.size(), enemyMap.size());
        for (int i = 0; i < loopSize; i++) {
            stpInfos["defender_" + std::to_string(i)].setPositionToDefend(enemyMap.begin()->second);
            enemyMap.erase(enemyMap.begin());
        }
        for (int i = loopSize; i < defenderNames.size(); i++) {
            // For each waller, stand in the right wall position and look at the ball
            auto positionToMoveTo = PositionComputations::getWallPosition(i, defenderNames.size() - enemyMap.size(), field, world);
            auto &wallerStpInfo = stpInfos["defender_" + std::to_string(i)];

            wallerStpInfo.setPositionToMoveTo(positionToMoveTo);
            wallerStpInfo.setAngle((world->getWorld()->getBall()->get()->position - field.leftGoalArea.rightLine().center()).angle());

            // If the waller is close to its target, ignore collisions
            constexpr double IGNORE_COLLISIONS_DISTANCE = 1.0;
            if ((wallerStpInfo.getRobot()->get()->getPos() - positionToMoveTo).length() < IGNORE_COLLISIONS_DISTANCE) {
                wallerStpInfo.setShouldAvoidOurRobots(false);
            }
        }
        return;
    };
    // Calculate the distance between the defenders and their enemies
    std::vector<std::vector<double>> cost_matrix;
    cost_matrix.resize(activeDefenderNames.size());
    int row_length = std::min(activeDefenderNames.size(), enemyMap.size());
    for (int i = 0; i < activeDefenderNames.size(); i++) {
        cost_matrix[i].resize(row_length);
        // Check if there are still enemies left
        if (enemyMap.empty()) continue;
        enemies.emplace_back(enemyMap.begin()->second);
        enemyMap.erase(enemyMap.begin());
    }
    // Calculate the optimal assignment of enemies to pass_defenders using the hungarian algorithm and set the position to defend for each
    // active pass defender
    std::vector<int> assignments;
    rtt::Hungarian::Solve(cost_matrix, assignments);
    int currentWallerIndex = 0;
    for (int i = 0; i < activeDefenderNames.size(); i++) {
        // If assignments is -1, it means the pass defender does not get an enemy assigned to it, because there are more pass defenders than enemies
        if (assignments[i] == -1) {
            // For each waller, stand in the right wall position and look at the ball
            auto positionToMoveTo = PositionComputations::getWallPosition(currentWallerIndex, activeDefenderNames.size() - enemies.size(), field, world);
            currentWallerIndex++;
            auto &wallerStpInfo = stpInfos[activeDefenderNames[i]];

            wallerStpInfo.setPositionToMoveTo(positionToMoveTo);
            wallerStpInfo.setAngle((world->getWorld()->getBall()->get()->position - field.leftGoalArea.rightLine().center()).angle());

            // If the waller is close to its target, ignore collisions
            constexpr double IGNORE_COLLISIONS_DISTANCE = 1.0;
            if ((wallerStpInfo.getRobot()->get()->getPos() - positionToMoveTo).length() < IGNORE_COLLISIONS_DISTANCE) {
                wallerStpInfo.setShouldAvoidOurRobots(false);
            }
        } else {
            if (enemies[assignments[i]].x > 0) {
                stpInfos[activeDefenderNames[i]].setPositionToDefend(Vector2(0, enemies[assignments[i]].y));
                stpInfos[activeDefenderNames[i]].setBlockDistance(BlockDistance::ROBOTRADIUS);
            } else {
                stpInfos[activeDefenderNames[i]].setPositionToDefend(enemies[assignments[i]]);
                stpInfos[activeDefenderNames[i]].setBlockDistance(BlockDistance::ROBOTRADIUS);
            }
        }
    }
}

void PositionComputations::calculateInfoForAttackers(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                     std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles, const Field &field,
                                                     world::World *world) noexcept {
    // List of all active attackers
    auto attackerNames = std::vector<std::string>{};
    for (int i = 0; i < world->getWorld()->getUs().size(); i++) {
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
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomRightGrid, gen::OffensivePosition, field, world));
    } else if (attackerNames.size() >= 3) {
        stpInfos["attacker_0"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topRightGrid, gen::OffensivePosition, field, world));
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleRightGrid, gen::AttackingPass, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomRightGrid, gen::OffensivePosition, field, world));
    }
    if (attackerNames.size() == 4) {
        stpInfos["attacker_3"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleMidGrid, gen::AttackingPass, field, world));
    } else if (attackerNames.size() == 5) {
        stpInfos["attacker_3"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::SafePass, field, world));
        stpInfos["attacker_4"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::OffensivePosition, field, world));
    } else if (attackerNames.size() >= 6) {
        stpInfos["attacker_3"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.topMidGrid, gen::OffensivePosition, field, world));
        stpInfos["attacker_4"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleMidGrid, gen::SafePass, field, world));
        stpInfos["attacker_5"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.bottomMidGrid, gen::OffensivePosition, field, world));
    }
}

void PositionComputations::calculateInfoForWallers(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                   std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles, const Field &field,
                                                   world::World *world) noexcept {
    // List of all active defender, such that we can defend the n closests enemies
    auto wallerNames = std::vector<std::string>{};
    for (int i = 0; i < world->getWorld()->getUs().size(); i++) {
        if (roles[i]->getName().find("waller") != std::string::npos) {
            wallerNames.emplace_back(roles[i]->getName());
        }
    }
    auto activeWallerNames = std::vector<std::string>{};
    for (auto name : wallerNames) {
        if (stpInfos[name].getRobot().has_value()) activeWallerNames.emplace_back(name);
    }

    for (int i = 0; i < activeWallerNames.size(); ++i) {
        // For each waller, stand in the right wall position and look at the ball
        auto positionToMoveTo = PositionComputations::getWallPosition(i, activeWallerNames.size(), field, world);
        auto &wallerStpInfo = stpInfos[activeWallerNames[i]];

        wallerStpInfo.setPositionToMoveTo(positionToMoveTo);
        wallerStpInfo.setAngle((world->getWorld()->getBall()->get()->position - field.leftGoalArea.rightLine().center()).angle());

        // If the waller is close to its target, ignore collisions
        constexpr double IGNORE_COLLISIONS_DISTANCE = 1.0;
        if ((wallerStpInfo.getRobot()->get()->getPos() - positionToMoveTo).length() < IGNORE_COLLISIONS_DISTANCE) {
            wallerStpInfo.setShouldAvoidOurRobots(false);
        }
    }
}

void PositionComputations::calculateInfoForFormation(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                     std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles, const Field &field,
                                                     world::World *world) noexcept {
    auto formationBackNames = std::vector<std::string>{};
    auto formationMidNames = std::vector<std::string>{};
    auto formationFrontNames = std::vector<std::string>{};
    for (int i = 0; i < world->getWorld()->getUs().size(); i++) {
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

    for (int i = 0; i < formationBackNames.size(); i++) {
        stpInfos[formationBackNames[i]].setPositionToMoveTo(Vector2{-width / 3.5, -height / 2 + height / (formationBackNames.size() + 1) * (i + 1)});
    }
    for (int i = 0; i < formationMidNames.size(); i++) {
        stpInfos[formationMidNames[i]].setPositionToMoveTo(Vector2{-width / 10, -height / 2 + height / (formationMidNames.size() + 1) * (i + 1)});
    }
    for (int i = 0; i < formationFrontNames.size(); i++) {
        stpInfos[formationFrontNames[i]].setPositionToMoveTo(Vector2{width / 8, -height / 2 + height / (formationFrontNames.size() + 1) * (i + 1)});
    }
}

void PositionComputations::calculateInfoForFormationOurSide(std::unordered_map<std::string, StpInfo> &stpInfos,
                                                            std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT> &roles, const Field &field,
                                                            world::World *world) noexcept {
    auto formationBackNames = std::vector<std::string>{};
    auto formationMidNames = std::vector<std::string>{};
    auto formationFrontNames = std::vector<std::string>{};
    for (int i = 0; i < world->getWorld()->getUs().size(); i++) {
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

    for (int i = 0; i < formationBackNames.size(); i++) {
        stpInfos[formationBackNames[i]].setPositionToMoveTo(Vector2{-width / 3, -height / 2 + height / (formationBackNames.size() + 1) * (i + 1)});
    }
    for (int i = 0; i < formationMidNames.size(); i++) {
        stpInfos[formationMidNames[i]].setPositionToMoveTo(Vector2{-width / 5, -height / 2 + height / (formationMidNames.size() + 1) * (i + 1)});
    }
    for (int i = 0; i < formationFrontNames.size(); i++) {
        stpInfos[formationFrontNames[i]].setPositionToMoveTo(Vector2{-width / 15, -height / 2 + height / (formationFrontNames.size() + 1) * (i + 1)});
    }
}

void PositionComputations::calculateInfoForPenalty(std::unordered_map<std::string, StpInfo> &stpInfos, const Field &field, world::World *world) noexcept {
    // During our penalty, all our robots should be behind the ball to not interfere.
    // Create a grid pattern of robots on our side of the field
    auto currentGameState = GameStateManager::getCurrentGameState().getStrategyName();
    constexpr double PENALTY_MARK_X = 0;
    int amountOfPassiveRobots = 0;
    if (currentGameState == "PenaltyUsPrepare") {
        constexpr double PENALTY_MARK_X = -2.0;
        amountOfPassiveRobots = world->getWorld()->getUs().size() - 2;
    } else if (currentGameState == "PenaltyThemPrepare") {
        constexpr double PENALTY_MARK_X = 2.0;
        amountOfPassiveRobots = world->getWorld()->getUs().size() - 1;
    } else {
        std::cout << currentGameState << std::endl;
        constexpr double PENALTY_MARK_X = 2.0;
    }
    // Determine where behind our robots have to stand
    auto ballPosition = world->getWorld()->getBall();
    // If there is no ball, use the default division A penalty mark position
    double ballX = ballPosition.has_value() ? ballPosition.value()->position.x : PENALTY_MARK_X;
    double limitX = std::min(ballX, PENALTY_MARK_X) - Constants::PENALTY_DISTANCE_BEHIND_BALL();

    // Then, figure out at what interval the robots will stand on a horizontal line
    double horizontalRange = std::fabs(field.playArea.left() - limitX);
    double horizontalHalfStep = horizontalRange / (5.0 * 2.0);  // 5 robots for stepSize, divided by 2 for half stepSize

    // Lastly, figure out vertical stepSize
    double verticalRange = std::fabs(field.leftDefenseArea.bottom() - field.playArea.bottom());
    double verticalHalfStep = verticalRange / (2.0 * 2.0);  // 2 rows, divided by 2 for half stepSize

    double startX = field.playArea.left() + horizontalHalfStep;
    double bottomY = field.playArea.bottom() + verticalHalfStep;
    double topY = bottomY + 2 * verticalHalfStep;

    const std::string formationPrefix = "formation_";

    /// Bottom row of 5 robots
    for (int i = 0; i < amountOfPassiveRobots / 2; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX + i * 2 * horizontalHalfStep, bottomY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.rightGoalArea.leftLine().center() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }

    /// Top row of 5 robots
    for (int i = amountOfPassiveRobots / 2; i < amountOfPassiveRobots; i++) {
        auto formationName = formationPrefix + std::to_string(i);
        auto position = Vector2(startX + (i - 5) * 2 * horizontalHalfStep, topY);
        stpInfos[formationName].setPositionToMoveTo(position);

        auto angleToGoal = (field.rightGoalArea.leftLine().center() - position).toAngle();
        stpInfos[formationName].setAngle(angleToGoal);
    }
}

}  // namespace rtt::ai::stp