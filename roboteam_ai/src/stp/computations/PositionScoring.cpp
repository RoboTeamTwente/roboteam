#include "stp/computations/PositionScoring.h"

#include "stp/computations/ComputationManager.h"
#include "stp/evaluations/position/GoalShotEvaluation.h"
#include "stp/evaluations/position/LineOfSightEvaluation.h"
#include "stp/evaluations/position/OpennessEvaluation.h"

namespace rtt::ai::stp {
gen::ScoredPosition PositionScoring::scorePosition(const Vector2 &position, const gen::ScoreProfile &profile, const Field &field, const world::World *world, uint8_t bias) {
    gen::PositionScores &scores = ComputationManager::calculatedScores[position];
    uint8_t positionScore = getScoreOfPosition(profile, position, scores, field, world);
    if (bias) positionScore = std::min(static_cast<uint32_t>(positionScore + bias), static_cast<uint32_t>(std::numeric_limits<uint8_t>::max()));  // Make sure we don't overflow
    return {position, positionScore};
}

uint8_t PositionScoring::getScoreOfPosition(const gen::ScoreProfile &profile, Vector2 position, gen::PositionScores &scores, const rtt::Field &field,
                                            const rtt::world::World *world) {
    double scoreTotal = 0;
    double weightTotal = 0;
    if (profile.weightGoalShot > 0) {
        scoreTotal += scores.scoreGoalShot.value_or(determineGoalShotScore(position, field, world, scores)) * profile.weightGoalShot;
        weightTotal += profile.weightGoalShot;
    }
    if (profile.weightLineOfSight > 0) {
        scoreTotal += scores.scoreLineOfSight.value_or(determineLineOfSightScore(position, world, scores)) * profile.weightLineOfSight;
        weightTotal += profile.weightLineOfSight;
    }
    if (profile.weightOpen > 0) {
        scoreTotal += scores.scoreOpen.value_or(determineOpenScore(position, field, world, scores)) * profile.weightOpen;
        weightTotal += profile.weightOpen;
    }
    return static_cast<uint8_t>(scoreTotal / weightTotal);
}

double PositionScoring::determineOpenScore(Vector2 &point, const rtt::Field &field, const rtt::world::World *world, gen::PositionScores &scores) {
    std::vector<double> enemyDistances;
    auto &them = world->getWorld()->getThem();
    enemyDistances.reserve(them.size());
    for (auto &enemyRobot : them) {
        enemyDistances.push_back(point.dist(enemyRobot->getPos()));
    }
    auto radius = field.playArea.width() / 4.0;
    return (scores.scoreOpen = stp::evaluation::OpennessEvaluation::metricCheck(enemyDistances, radius)).value();
}

double PositionScoring::determineLineOfSightScore(Vector2 &point, const rtt::world::World *world, gen::PositionScores &scores) {
    Vector2 ballPos = world->getWorld().value()->getBall()->get()->position;
    double pointDistance = ballPos.dist(point);
    std::vector<double> enemyDistancesToBall;
    std::vector<double> enemyAnglesToBallvsPoint;
    auto &them = world->getWorld()->getThem();
    enemyDistancesToBall.reserve(them.size());
    enemyAnglesToBallvsPoint.reserve(them.size());
    for (auto &enemyRobot : them) {
        enemyDistancesToBall.push_back(ballPos.dist(enemyRobot->getPos()));
        enemyAnglesToBallvsPoint.push_back((point - ballPos).toAngle().shortestAngleDiff((enemyRobot->getPos() - ballPos)));
    }
    return (scores.scoreLineOfSight = stp::evaluation::LineOfSightEvaluation::metricCheck(pointDistance, enemyDistancesToBall, enemyAnglesToBallvsPoint)).value();
}

double PositionScoring::determineGoalShotScore(Vector2 &point, const rtt::Field &field, const rtt::world::World *world, gen::PositionScores &scores) {
    double visibility = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, point, world->getWorld().value(), -1, true) / 100;
    double goalAngle = FieldComputations::getTotalGoalAngle(field, false, point);
    double distanceToGoal = point.dist(field.rightDefenseArea.rightLine().center());
    double minDistanceToGoal = field.rightDefenseArea.rightLine().center().dist(field.rightDefenseArea.leftLine().center());
    double maxDistanceToGoal = field.rightDefenseArea.rightLine().center().dist(field.playArea.topLeft());
    double normalizedDistanceToGoal = (distanceToGoal - minDistanceToGoal) / (maxDistanceToGoal - minDistanceToGoal);

    // The goal angle from right in front of their defense area- i.e. the "best" goal angle
    double maxGoalAngle = FieldComputations::getTotalGoalAngle(field, false, field.rightDefenseArea.leftLine().center());
    return (scores.scoreGoalShot = stp::evaluation::GoalShotEvaluation::metricCheck(visibility, goalAngle / maxGoalAngle, normalizedDistanceToGoal)).value();
}
}  // namespace rtt::ai::stp