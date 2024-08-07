#include "world/FieldComputations.h"

#include <roboteam_utils/Shadow.h>

#include "utilities/GameSettings.h"
#include "utilities/GameStateManager.hpp"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai {

std::tuple<double, double> FieldComputations::getDefenseAreaMargin() {
    double theirDefenseAreaMargin = constants::ROBOT_RADIUS * 1.4 + constants::GO_TO_POS_ERROR_MARGIN;
    double ourDefenseAreaMargin = -constants::ROBOT_RADIUS * 0.9 + constants::GO_TO_POS_ERROR_MARGIN;

    RuleSetName ruleSetTitle = GameStateManager::getCurrentGameState().getRuleSet().getTitle();
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();

    if (ruleSetTitle == RuleSetName::STOP || currentGameState == RefCommand::DIRECT_FREE_THEM || currentGameState == RefCommand::KICKOFF_THEM ||
        currentGameState == RefCommand::DIRECT_FREE_US) {
        theirDefenseAreaMargin += 0.3;
    }

    return std::make_tuple(theirDefenseAreaMargin, ourDefenseAreaMargin);
}

bool FieldComputations::getBallAvoidance() {
    RuleSetName ruleSetTitle = GameStateManager::getCurrentGameState().getRuleSet().getTitle();
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();
    if (ruleSetTitle == RuleSetName::STOP || currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::KICKOFF_THEM ||
        currentGameState == RefCommand::DIRECT_FREE_THEM) {
        return true;
    }

    return false;
}

Vector2 FieldComputations::getBallPositionAtTime(const rtt::world::ball::Ball &ball, double time) {
    const double initialVelocity = ball.velocity.length();
    const double frictionCoefficient = GameSettings::getRobotHubMode() == net::RobotHubMode::SIMULATOR ? ai::constants::SIMULATION_FRICTION : ai::constants::REAL_FRICTION;
    double timeToStop = initialVelocity / frictionCoefficient;
    if (time > timeToStop) {
        time = timeToStop;
    }
    double distance = initialVelocity * time - 0.5 * frictionCoefficient * time * time;
    auto expectedEndPosition = ball.position + ball.velocity.stretchToLength(distance);
    return expectedEndPosition;
}

double FieldComputations::getBallTimeAtPosition(const rtt::world::ball::Ball &ball, const Vector2 &targetPoint) {
    const double frictionCoefficient = GameSettings::getRobotHubMode() == net::RobotHubMode::SIMULATOR ? ai::constants::SIMULATION_FRICTION : ai::constants::REAL_FRICTION;

    Vector2 direction = targetPoint - ball.position;
    Vector2 velocityUnit = ball.velocity.normalize();
    Vector2 projectedPoint = ball.position + velocityUnit * (direction.dot(velocityUnit));

    double distanceToTarget = (projectedPoint - ball.position).length();

    // distance = initialVelocity * time - 0.5 * frictionCoefficient * time^2
    double a = -0.5 * frictionCoefficient;
    double b = ball.velocity.length();
    double c = -distanceToTarget;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return std::numeric_limits<double>::infinity();
    }

    double root1 = (-b + std::sqrt(discriminant)) / (2 * a);
    double root2 = (-b - std::sqrt(discriminant)) / (2 * a);
    double minPositiveRoot = std::min({root1, root2});

    return minPositiveRoot < 0 ? std::numeric_limits<double>::infinity() : minPositiveRoot;
}

bool FieldComputations::pointIsValidPosition(const rtt::Field &field, const Vector2 &point, stp::AvoidObjects avoidObjects, double fieldMargin) {
    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = getDefenseAreaMargin();
    if (avoidObjects.shouldAvoidOutOfField && !field.playArea.contains(point, fieldMargin)) return false;
    if (avoidObjects.shouldAvoidOurDefenseArea && (field.leftDefenseArea.contains(point, ourDefenseAreaMargin))) return false;
    if (avoidObjects.shouldAvoidTheirDefenseArea && (field.rightDefenseArea.contains(point, theirDefenseAreaMargin))) return false;
    auto leftGoalTopPost = field.leftGoalArea.topLine();
    auto leftGoalBottomPost = field.leftGoalArea.bottomLine();
    auto rightGoalTopPost = field.rightGoalArea.topLine();
    auto rightGoalBottomPost = field.rightGoalArea.bottomLine();
    if (avoidObjects.shouldAvoidGoalPosts) {
        if (leftGoalTopPost.distanceToLine(point) < constants::ROBOT_RADIUS || leftGoalBottomPost.distanceToLine(point) < constants::ROBOT_RADIUS ||
            rightGoalTopPost.distanceToLine(point) < constants::ROBOT_RADIUS || rightGoalBottomPost.distanceToLine(point) < constants::ROBOT_RADIUS) {
            return false;
        }
    }
    return true;
}

double FieldComputations::getTotalGoalAngle(const rtt::Field &field, bool ourGoal, const Vector2 &point) {
    LineSegment goal = ourGoal ? field.leftGoalArea.rightLine() : field.rightGoalArea.leftLine();
    Angle angleLeft = Angle(goal.start - point);
    Angle angleRight = Angle(goal.end - point);
    return angleLeft.shortestAngleDiff(angleRight);
}

double FieldComputations::getPercentageOfGoalVisibleFromPoint(const rtt::Field &field, bool ourGoal, const Vector2 &point, rtt::world::view::WorldDataView world, int id,
                                                              bool ourTeam) {
    double goalWidth = field.leftGoalArea.height();
    double blockadeLength = 0;
    auto &robots = ourTeam ? world.getThem() : world.getUs();
    for (auto const &blockade : getBlockadesMappedToGoal(field, ourGoal, point, robots, id, ourTeam)) {
        blockadeLength += blockade.start.dist(blockade.end);
    }
    return fmax(100 - blockadeLength / goalWidth * 100, 0.0);
}

std::vector<LineSegment> FieldComputations::getVisiblePartsOfGoal(const rtt::Field &field, bool ourGoal, const Vector2 &point,
                                                                  const std::vector<rtt::world::view::RobotView> &robots) {
    // TODO: improve when it takes our/their robots into account
    std::vector<LineSegment> blockades = getBlockadesMappedToGoal(field, ourGoal, point, robots, -1, true);
    LineSegment goalSide = ourGoal ? field.leftGoalArea.rightLine() : field.rightGoalArea.leftLine();
    double goalX = goalSide.start.x;  // The x-coordinate of the entire goal line (all vectors on this line have the same x-coordinate).
    double upperGoalY = goalSide.end.y;
    double lowerY = goalSide.start.y;
    std::vector<LineSegment> visibleParts = {};

    // The obstacles are sorted on their smallest y value. We start from the lowest goal side at the start as lowerY value and everytime we add a vector from the lowest goalside to
    // the lowest part of the obstacle and we remember the upper part of the obstacle. That upper part is stored as the lowerY value again and we can repeat the same process.
    for (auto const &blockade : blockades) {
        auto lowerbound = blockade.start.y;

        // If the lowerbound is the same as the lowerY value then the visible part has a length of 0 and we don't care about it. Originally used to be != but floating point errors
        // are tears, i.e. rounding of floating points might turn two same float values to different values.
        if (fabs(lowerbound - lowerY) > NEGLIGIBLE_LENGTH) {
            visibleParts.emplace_back(LineSegment(Vector2(goalX, lowerY), Vector2(goalX, lowerbound)));
        }
        lowerY = blockade.end.y;
    }

    // If the last lowerY value is the same as the upper goal side then the last visible part has a length of 0 and we don't care about it.
    if (fabs(lowerY - upperGoalY) > NEGLIGIBLE_LENGTH) {
        visibleParts.emplace_back(LineSegment(Vector2(goalX, lowerY), Vector2(goalX, upperGoalY)));
    }
    return visibleParts;
}

double FieldComputations::getDistanceToGoal(const rtt::Field &field, bool ourGoal, const Vector2 &point) {
    return ourGoal ? field.leftGoalArea.rightLine().distanceToLine(point) : field.rightGoalArea.leftLine().distanceToLine(point);
}

std::shared_ptr<Vector2> FieldComputations::lineIntersectionWithDefenseArea(const rtt::Field &field, bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd, double margin,
                                                                            bool ignoreGoalLine) {
    auto defenseArea = getDefenseArea(field, ourGoal, margin, field.boundaryWidth);

    std::vector<Vector2> intersections;
    if (!ignoreGoalLine) {
        intersections = defenseArea.intersections({lineStart, lineEnd});
    } else {
        auto defenseAreaVertices = defenseArea.vertices;
        // Loop over the all lines of the defense area except the goal line and check for intersections
        for (size_t i = 0; i < defenseAreaVertices.size() - 1; i++) {
            auto intersection = LineSegment(defenseAreaVertices[i], defenseAreaVertices[i + 1]).intersects({lineStart, lineEnd});
            if (intersection) intersections.push_back(intersection.value());
        }
    }

    if (intersections.size() == 1) {
        return std::make_shared<Vector2>(intersections.at(0));
    } else if (intersections.size() == 2) {
        double distanceFirstIntersection = lineStart.dist(intersections.at(0));
        double distanceSecondIntersection = lineStart.dist(intersections.at(1));
        return std::make_shared<Vector2>(distanceFirstIntersection < distanceSecondIntersection ? intersections.at(0) : intersections.at(1));
    } else {
        return nullptr;
    }
}

std::optional<Vector2> FieldComputations::lineIntersectionWithField(const rtt::Field &field, const Vector2 &lineStart, const Vector2 &lineEnd, double margin) {
    auto fieldEdges = getFieldEdge(field, margin);
    auto intersections = fieldEdges.intersections({lineStart, lineEnd});

    if (intersections.size() == 1) {
        return intersections.at(0);
    } else if (intersections.size() == 2) {
        double distanceFirstIntersection = lineStart.dist(intersections.at(0));
        double distanceSecondIntersection = lineStart.dist(intersections.at(1));
        return distanceFirstIntersection < distanceSecondIntersection ? intersections.at(0) : intersections.at(1);
    } else {
        return std::nullopt;
    }
}

// True standard which mean field.boundaryWidth is used otherwise margin is used
Polygon FieldComputations::getDefenseArea(const rtt::Field &field, bool ourDefenseArea, double margin, double backMargin) {
    Vector2 belowGoal = ourDefenseArea ? field.leftDefenseArea.bottomLeft() : field.rightDefenseArea.bottomRight();
    Vector2 aboveGoal = ourDefenseArea ? field.leftDefenseArea.topLeft() : field.rightDefenseArea.topRight();
    Vector2 bottomPenalty = ourDefenseArea ? field.leftDefenseArea.bottomRight() : field.rightDefenseArea.bottomLeft();
    Vector2 topPenalty = ourDefenseArea ? field.leftDefenseArea.topRight() : field.rightDefenseArea.topLeft();

    if (aboveGoal.y < belowGoal.y) {
        std::swap(aboveGoal, belowGoal);
    }
    if (topPenalty.y < bottomPenalty.y) {
        std::swap(topPenalty, bottomPenalty);
    }

    if (ourDefenseArea) {
        if (aboveGoal.x > topPenalty.x) {
            std::swap(aboveGoal, topPenalty);
        }
        if (belowGoal.x > bottomPenalty.x) {
            std::swap(belowGoal, bottomPenalty);
        }
    } else {
        if (aboveGoal.x < topPenalty.x) {
            std::swap(aboveGoal, topPenalty);
        }
        if (belowGoal.x < bottomPenalty.x) {
            std::swap(belowGoal, bottomPenalty);
        }
    }

    belowGoal = ourDefenseArea ? belowGoal + Vector2(-backMargin, -margin) : belowGoal + Vector2(backMargin, -margin);
    aboveGoal = ourDefenseArea ? aboveGoal + Vector2(-backMargin, margin) : aboveGoal + Vector2(backMargin, margin);
    bottomPenalty = ourDefenseArea ? bottomPenalty + Vector2(margin, -margin) : bottomPenalty + Vector2(-margin, -margin);
    topPenalty = ourDefenseArea ? topPenalty + Vector2(margin, margin) : topPenalty + Vector2(-margin, margin);

    std::vector<Vector2> defenseArea = {belowGoal, bottomPenalty, topPenalty, aboveGoal};
    return {defenseArea};
}

Polygon FieldComputations::getGoalArea(const rtt::Field &field, bool ourGoal, double margin, bool hasBackMargin) {
    double goalDepth = hasBackMargin ? field.leftGoalArea.width() + margin : field.leftGoalArea.width();
    Vector2 outerBottomGoal = ourGoal ? field.leftGoalArea.bottomRight() + Vector2(margin, -margin) : field.rightGoalArea.bottomLeft() + Vector2(-margin, -margin);
    Vector2 innerBottomGoal = ourGoal ? field.leftGoalArea.bottomRight() + Vector2(-goalDepth, -margin) : field.rightGoalArea.bottomLeft() + Vector2(goalDepth, -margin);
    Vector2 innerTopGoal = ourGoal ? field.leftGoalArea.topRight() + Vector2(-goalDepth, margin) : field.rightGoalArea.topLeft() + Vector2(goalDepth, margin);
    Vector2 outerTopGoal = ourGoal ? field.leftGoalArea.topRight() + Vector2(margin, margin) : field.rightGoalArea.topLeft() + Vector2(-margin, margin);

    std::vector<Vector2> goalArea = {outerBottomGoal, innerBottomGoal, innerTopGoal, outerTopGoal};
    return Polygon(goalArea);
}

Polygon FieldComputations::getFieldEdge(const rtt::Field &field, double margin) {
    std::vector<Vector2> fieldEdge = {field.playArea.bottomLeft() + Vector2(-margin, -margin), field.playArea.topLeft() + Vector2(-margin, margin),
                                      field.playArea.topRight() + Vector2(margin, margin), field.playArea.bottomRight() + Vector2(margin, -margin)};
    return Polygon(fieldEdge);
}

std::vector<LineSegment> FieldComputations::getBlockadesMappedToGoal(const rtt::Field &field, bool ourGoal, const Vector2 &point,
                                                                     const std::vector<rtt::world::view::RobotView> &robots, int id, bool ourTeam) {
    std::vector<LineSegment> blockades = {};
    const double robotRadius = constants::ROBOT_RADIUS + constants::BALL_RADIUS;
    LineSegment goalSide = ourGoal ? field.leftGoalArea.rightLine() : field.rightGoalArea.leftLine();
    for (auto const &robot : robots) {
        std::optional<LineSegment> blockade = robotBlockade(ourGoal, point, id, ourTeam, robot, robotRadius, goalSide);
        if (blockade.has_value()) {
            blockades.emplace_back(blockade.value());
        }
    }
    return mergeBlockades(blockades);
}

std::optional<LineSegment> FieldComputations::robotBlockade(bool ourGoal, const Vector2 &point, int id, bool ourTeam, const rtt::world::view::RobotView robot,
                                                            const double robotRadius, LineSegment goalSide) {
    // Discard the robot if it belong to the same team or if it has the given id.
    if (robot->getId() == id || robot->getTeam() == (ourTeam ? rtt::world::Team::us : rtt::world::Team::them)) return {};

    // Discard already the robot if it is not between the goal and point, or if the robot is standing on this point.
    double lenToBot = (point - robot->getPos()).length();
    bool isRobotItself = lenToBot <= robotRadius;
    bool isInPotentialBlockingZone = ourGoal ? robot->getPos().x < point.x + robotRadius : robot->getPos().x > point.x - robotRadius;
    if (isRobotItself || !isInPotentialBlockingZone) return {};

    // Compute the shadow caused by the robot on the goal side
    double theta = asin(robotRadius / lenToBot);
    double length = sqrt(lenToBot * lenToBot - robotRadius * robotRadius);
    Vector2 lowerSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->getPos()) - point).angle() - theta);
    Vector2 upperSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->getPos()) - point).angle() + theta);
    return Shadow::shadow(point, LineSegment(lowerSideOfRobot, upperSideOfRobot), goalSide, NEGLIGIBLE_LENGTH);
}

std::vector<LineSegment> FieldComputations::mergeBlockades(std::vector<LineSegment> blockades) {
    /* If two blockades intersect (in this case, overlap), we take the beginning of the first obstacle and the end of the second obstacle, and put them back in the front of the
     * obstacles vector. The second element gets erased. If they don't intersect, try the next two obstacles. Repeat this procedure until no overlaps are left. */
    std::sort(blockades.begin(), blockades.end(), [](const LineSegment &a, const LineSegment &b) { return a.start.y < b.start.y; });
    int iterator = 0;
    while (iterator < static_cast<int>(blockades.size()) - 1) {
        LineSegment &firstBlockade = blockades.at(iterator);
        LineSegment &secondBlockade = blockades.at(iterator + 1);
        if (firstBlockade.end.y >= secondBlockade.start.y) {
            // If the first two elements intersects, then merge these blockades into 1 single blockade.
            auto upperbound = fmax(firstBlockade.end.y, secondBlockade.end.y);
            auto newBlockade = LineSegment(firstBlockade.start, Vector2(firstBlockade.start.x, upperbound));
            blockades.erase(blockades.begin() + iterator + 1);
            blockades.at(iterator) = newBlockade;
        } else {
            iterator++;
        }
    }
    return blockades;
}

Vector2 FieldComputations::projectPointInField(const Field &field, Vector2 point, double margin) {
    Vector2 projectedPoint;
    projectedPoint.x = std::clamp(point.x, field.playArea.left() + margin, field.playArea.right() - margin);
    projectedPoint.y = std::clamp(point.y, field.playArea.bottom() + margin, field.playArea.top() - margin);
    return projectedPoint;
}

Vector2 FieldComputations::projectPointOutOfDefenseArea(const Field &field, Vector2 point, bool outOurDefenseArea, bool outTheirDefenseArea) {
    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = getDefenseAreaMargin();
    if ((!field.rightDefenseArea.contains(point, theirDefenseAreaMargin) || !outTheirDefenseArea) &&
        (!field.leftDefenseArea.contains(point, ourDefenseAreaMargin) || !outOurDefenseArea))
        return point;

    // If the point is not in the field yet, project it into the field
    if (!field.playArea.contains(point)) {
        point = projectPointInField(field, point);
    }

    // Calculate how far the point is from the defense area border in the x and y direction
    double xDiff;
    double yDiff;
    if (field.leftDefenseArea.contains(point, ourDefenseAreaMargin)) {
        xDiff = (field.leftDefenseArea.right() + ourDefenseAreaMargin) - point.x;
        yDiff = point.y > 0 ? (field.leftDefenseArea.top() + ourDefenseAreaMargin) - point.y : (field.leftDefenseArea.bottom() - ourDefenseAreaMargin) - point.y;
    } else if (field.rightDefenseArea.contains(point, theirDefenseAreaMargin)) {
        xDiff = (field.rightDefenseArea.left() - theirDefenseAreaMargin) - point.x;
        yDiff = point.y > 0 ? (field.rightDefenseArea.top() + theirDefenseAreaMargin) - point.y : (field.rightDefenseArea.bottom() - theirDefenseAreaMargin) - point.y;
    } else
        return point;  // In case it is in neither defense area, just return the point

    if (fabs(xDiff) < fabs(yDiff)) return {point.x + xDiff, point.y};
    return {point.x, point.y + yDiff};
}

Vector2 FieldComputations::projectPointToValidPosition(const Field &field, Vector2 point, stp::AvoidObjects avoidObjects) {
    Vector2 projectedPos = point;
    if (avoidObjects.shouldAvoidOutOfField) projectedPos = projectPointInField(field, projectedPos);
    projectedPos = projectPointOutOfDefenseArea(field, projectedPos, avoidObjects.shouldAvoidOurDefenseArea, avoidObjects.shouldAvoidTheirDefenseArea);
    if (avoidObjects.shouldAvoidGoalPosts) {
        auto leftGoalTopPost = field.leftGoalArea.topLine();
        auto leftGoalBottomPost = field.leftGoalArea.bottomLine();
        auto rightGoalTopPost = field.rightGoalArea.topLine();
        auto rightGoalBottomPost = field.rightGoalArea.bottomLine();
        if (leftGoalTopPost.distanceToLine(point) < constants::ROBOT_RADIUS) {
            projectedPos = {leftGoalTopPost.end.x + constants::ROBOT_RADIUS, projectedPos.y};
        } else if (leftGoalBottomPost.distanceToLine(point) < constants::ROBOT_RADIUS) {
            projectedPos = {leftGoalBottomPost.start.x + constants::ROBOT_RADIUS, projectedPos.y};
        } else if (rightGoalTopPost.distanceToLine(point) < constants::ROBOT_RADIUS) {
            projectedPos = {rightGoalTopPost.start.x - constants::ROBOT_RADIUS, projectedPos.y};
        } else if (rightGoalBottomPost.distanceToLine(point) < constants::ROBOT_RADIUS) {
            projectedPos = {rightGoalBottomPost.end.x - constants::ROBOT_RADIUS, projectedPos.y};
        }
    }
    return projectedPos;
}

Vector2 FieldComputations::projectPointIntoFieldOnLine(const Field &field, Vector2 point, Vector2 p1, Vector2 p2, double fieldMargin) {
    auto projectedPos = LineSegment(p1, p2).project(point);
    if (field.playArea.contains(projectedPos, fieldMargin)) return projectedPos;

    auto intersection_lhs = FieldComputations::lineIntersectionWithField(field, projectedPos, p1, fieldMargin);
    auto intersection_rhs = FieldComputations::lineIntersectionWithField(field, projectedPos, p2, fieldMargin);

    // If there is no point  on this line inside the field, project the position into the field and return it
    if (!intersection_lhs && !intersection_rhs) return projectPointInField(field, projectedPos, fieldMargin);
    double dist_lhs = intersection_lhs ? point.dist(*intersection_lhs) : std::numeric_limits<double>::max();
    double dist_rhs = intersection_rhs ? point.dist(*intersection_rhs) : std::numeric_limits<double>::max();
    return (dist_lhs < dist_rhs ? *intersection_lhs : *intersection_rhs);  // return the intersection closest to the point
}

Vector2 FieldComputations::projectPointToValidPositionOnLine(const Field &field, Vector2 point, Vector2 p1, Vector2 p2, double fieldMargin) {
    auto [theirDefenseAreaMargin, ourDefenseAreaMargin] = getDefenseAreaMargin();
    auto pointProjectedInField = projectPointIntoFieldOnLine(field, point, p1, p2, fieldMargin);

    bool ourGoal;   // Which goal's defense area the projected point is in
    double margin;  // The margin to be used for the defense area- set to ourDefenseMargin or theirDefenseAreaMargin depending on where the projected pos is
    if (field.leftDefenseArea.contains(pointProjectedInField, ourDefenseAreaMargin)) {
        margin = ourDefenseAreaMargin;
        ourGoal = true;
    } else if (field.rightDefenseArea.contains(pointProjectedInField, theirDefenseAreaMargin)) {
        margin = theirDefenseAreaMargin;
        ourGoal = false;
    } else {
        return pointProjectedInField;  // If the projected position is not in a defense area, return it
    }

    std::vector<Vector2> intersections;
    auto defenseAreaVertices = getDefenseArea(field, ourGoal, margin, 0).vertices;
    // Loop over the all lines of the defense area except the goal line and check for intersections
    for (size_t i = 0; i < defenseAreaVertices.size() - 1; i++) {
        auto intersection = LineSegment(defenseAreaVertices[i], defenseAreaVertices[i + 1]).intersects({p1, p2});
        if (intersection) intersections.push_back(intersection.value());
    }

    // If there is no point  on this line outside the defense area, project the position out of the defense area and return it
    if (intersections.empty()) return projectPointOutOfDefenseArea(field, pointProjectedInField);

    // Return the intersection closest to the given point
    return *std::min_element(intersections.begin(), intersections.end(), [&point](auto lhs, auto rhs) { return point.dist(lhs) < point.dist(rhs); });
}
};  // namespace rtt::ai
