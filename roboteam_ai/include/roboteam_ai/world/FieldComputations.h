#ifndef ROBOTEAM_AI_FIELDCOMPUTATIONS_H
#define ROBOTEAM_AI_FIELDCOMPUTATIONS_H

#include <proto/messages_robocup_ssl_geometry.pb.h>
#include <roboteam_utils/Polygon.h>

#include <cmath>
#include <mutex>
#include <roboteam_utils/Field.hpp>

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"
#include "utilities/StpInfoEnums.h"
#include "views/WorldDataView.hpp"

namespace rtt::world::view {
class WorldDataView;
class RobotView;
}  // namespace rtt::world::view

namespace rtt::ai {

/**
 * The Field Computation class does all kind of computations on the Field based on the Field class variables.
 */
class FieldComputations {
   private:
    static constexpr double NEGLIGIBLE_LENGTH = 0.000001; /**< If a line length is below or equal to this threshold then is it neglected during determining the blockades. */
    static constexpr double PROJECTION_MARGIN = 0.02;     /**< Some extra padding is added when projecting positions,so that the projected position is not on lines/intersections */

   public:
    /**
     * @brief Get the margin that should be used for the defense area avoidance
     * @return A tuple with the margin that should be used for the defense area avoidance
     */
    static std::tuple<double, double> getDefenseAreaMargin();

    /**
     * @brief Get ball avoidance based on current game state
     * @return A bool indicating whether ball avoidance should be used or not
     */
    static bool getBallAvoidance();

    /**
     * @brief Get the expected ball position after a certain amount of time
     * @param ball The current ball position
     * @param time The time in seconds after which the expected ball position should be returned
     */
    static Vector2 getBallPositionAtTime(const rtt::world::ball::Ball &ball, double time);

    /**
     * @brief Get the expected time it takes for the ball to reach a certain position
     * @param ball The current ball position
     * @param point The point for which the time is calculated
     */
    static double getBallTimeAtPosition(const rtt::world::ball::Ball &ball, const Vector2 &point);

    /**
     * @brief Check whether a given point is a valid position given which parts of the field should be avoided (note that shouldAvoidBall is not taken into consideration)
     * @param field The field class which is used to determine the boundaries of the field.
     * @param point The point for which it is checked whether it is valid or not
     * @param avoidObjects Struct indicating which areas of the field should be avoided. Defaults to avoid entering the defense area and leaving the field
     * @param fieldMargin The outwards margin in which the field area will get expanded/shrunk in all directions.
     * @return True if the point is not within any area that has to be avoided according to avoidObjects, note that this does not take ball avoidance into consideration
     */
    static bool pointIsValidPosition(const rtt::Field &field, const Vector2 &point, stp::AvoidObjects avoidObjects = {}, double fieldMargin = 0.0);

    /**
     * @brief Project given position to within the field with a certain margin
     * @param field The field class used to determine where the field lines are
     * @param point The position to be projected to within the field
     * @param margin The margin that should be used when calculating the new position. The position will have a minimum of this distance to the field lines
     * @return The position closest to the given point that is inside the field with the given margin. If the given point is already inside the field, the same point is returned
     */
    static Vector2 projectPointInField(const rtt::Field &field, Vector2 point, double margin = 0);

    /**
     * @brief Project given position to outside the defense areas with a certain margin
     * @param field The field class used to determine where the defense area is
     * @param point The position to be projected to outside of the defense area
     * @return The position closest to the given point that is outside of either defense area with the given margin and within the field.
     * Note that the returned position will always be within the field, even if this is not the closest position that is out of the defense area.
     * If the given point is already out of the defense area, this same point is returned.
     */
    static Vector2 projectPointOutOfDefenseArea(const rtt::Field &field, Vector2 point);

    /**
     * @brief Project given position to a valid position given which parts of the field should be avoided (note that shouldAvoidBall is not taken into consideration)
     * @param field The field class which is used to determine the boundaries of the field.
     * @param point The point to be projected to a valid position
     * @param avoidObjects Struct indicating which areas of the field should be avoided. Defaults to avoid entering the defense area and leaving the field
     * @return The position closest to the given point that fulfills the criteria set in avoidObjects (except ball avoidance)
     */
    static Vector2 projectPointToValidPosition(const rtt::Field &field, Vector2 point, stp::AvoidObjects avoidObjects = {});

    /**
     * @brief Projects the given point into the field on a line between two given points.
     * @param field The field class which is used to determine the boundaries of the field.
     * @param point The point to be projected into the field
     * @param p1 First point on the line
     * @param p2 Second point on the line
     * @param fieldMargin The outwards margin in which the field area will get expanded/shrunk in all directions. A positive value means that the field area will be
     * expanded, a negative value means that the field area will be shrunk.
     * @return The position in the field closest to p1 on the line between p1 and p2.
     * If no such position can be found, return the closest position in the field from the given point (but not on the given line).
     */
    static Vector2 projectPointIntoFieldOnLine(const Field &field, Vector2 point, Vector2 p1, Vector2 p2, double fieldMargin = 0.0);

    /**
     * @brief Projects the given point to a valid position, as defined by avoidObjects, on a line between two given points.
     * @param field The field class which is used to determine the boundaries of the field.
     * @param point The point to be projected to a valid position
     * @param p1 First point on the line
     * @param p2 Second point on the line
     * @param fieldMargin The outwards margin in which the field area will get expanded/shrunk in all directions. A positive value means that the field area will be
     * expanded, a negative value means that the field area will be shrunk.
     * @return The closest valid position to the given point on the line between p1 and p2.
     * If no such position can be found, return the closest valid position to the given point (but not on the given line).
     */
    static Vector2 projectPointToValidPositionOnLine(const Field &field, Vector2 point, Vector2 p1, Vector2 p2, double fieldMargin = 0);

    /**
     * @brief Get the percentage of goal visible from a given point, i.e. how much of the goal can be reached by directly shooting a ball over the ground from a given point without
     * hitting any robot from a given team.
     * @param field The field class used to determine where the goals are.
     * @param ourGoal True if we want to compute this for our goal, false if we want to compute it for the opponents goal.
     * @param point The point from which it is checked how much of the goal is visible.
     * @param world Data about the world which is used to determine the locations of all robots.
     * @param id The id of the robot which is not considered as blockade. Set this value to -1 if you do exclude a robot as blockade (by default no robot is excluded as blockade).
     * @param ourTeam True if our robots get excluded as blockades. False if all opponents robots get excluded as blockades (by default the opponents robots get excluded as
     * blockades).
     * @return The percentage of the goal visible, which is a double value between 0.0 and 100.0 including both 0.0 and 100.0.
     */
    static double getPercentageOfGoalVisibleFromPoint(const rtt::Field &field, bool ourGoal, const Vector2 &point, rtt::world::view::WorldDataView world, int id = -1,
                                                      bool ourTeam = false);

    /**
     * @brief Look at the overloaded function getVisiblePartsOfGoal(const Field &field, bool ourGoal, const Vector2 &point, world::view::WorldDataView &world) for the corresponding
     * documentation.
     * @param field The current field.
     * @param ourGoal Indicates whether our goal should be used for the visibility.
     * @param point The position from which we want to calculate the visibility.
     * @param robots A list of all robots that could possibly block the goal.
     * @return A vector with all parts of the goal that are visible from the given position.
     * @cite getVisiblePartsOfGoal(const Field &field, bool ourGoal, const Vector2 &point, world::view::WorldDataView &world)
     */
    static std::vector<LineSegment> getVisiblePartsOfGoal(const rtt::Field &field, bool ourGoal, const Vector2 &point, const std::vector<rtt::world::view::RobotView> &robots);

    /**
     * @brief Compute the Euclidean distance from a given point to the closest point on the goal.
     * @param field The field used to determine where the goals are.
     * @param ourGoal True if the distance towards our goal is computed, false if the distance towards the opponents goal is computed.
     * @param point The given point from which the distance is computed.
     * @return The Euclidean distance to the closest point on the goal from the given point.
     */
    static double getDistanceToGoal(const rtt::Field &field, bool ourGoal, const Vector2 &point);

    /**
     * @brief Determine the intersection between a LineSegment and the boundary of the defence area and return the intersection point closest to the start of the line (if the
     * LineSegment does not intersect then return a null pointer).
     * @param field The field used to determine where the defence area is located.
     * @param ourGoal True if we want to compute the intersection with our defence area, false if we compute this for the opponents defence area.
     * @param lineStart The location of the start of the LineSegment.
     * @param lineEnd The location of the end of the LineSegment.
     * @param margin The outwards margin in which the defence area will be expanded/shrank in all directions (except maybe for the goal side). A positive value means that it will
     * be expanded, a negative value means that it will be shrank (if unset then it will be neither expanded/shrank).
     * @param ignoreGoalLine If true, do not count intersections with the goal line (the line of the defense area that overlaps with the left line of the field).
     * @return The closest intersection point to the start of the LineSegment. In case of no intersection point return a null pointer.
     */
    static std::shared_ptr<Vector2> lineIntersectionWithDefenseArea(const rtt::Field &field, bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd, double margin,
                                                                    bool ignoreGoalLine = false);

    /**
     * @brief Determine the intersection between a LineSegment and the field lines and return the intersection point closest to the start of the line (if the LineSegment
     * does not intersect then return a null_opt).
     * @param field The field used to determine where the defence area is located.
     * @param lineStart The location of the start of the LineSegment.
     * @param lineEnd The location of the end of the LineSegment.
     * @param margin The outwards margin in which the field area will be expanded/shrinked in all directions (except maybe for the goal side). A positive value means that it will
     * be expanded, a negative value means that it will be shrinked (if unset then it will be neither expanded/shrinked).
     * @return The closest intersection point to the start of the LineSegment. In case of no intersection point return a null optional.
     */
    static std::optional<Vector2> lineIntersectionWithField(const rtt::Field &field, const Vector2 &lineStart, const Vector2 &lineEnd, double margin);

    /**
     * @brief Compute the total angle a given point makes with the goal, i.e. you create a triangle using this point and both upperside and lowerside of the goal and compute the
     * angle at this given point. Warning: this function does not work for points on the start & end points of the goal.
     * @param field The field used to determine where the goals are.
     * @param ourGoal True if the angle between the point and our goal is computed, false if we compute this for the opponents goal.
     * @param point The given point from which the angle is computed.
     * @return The angle in radians that this point makes with the given goal.
     */
    static double getTotalGoalAngle(const rtt::Field &field, bool ourGoal, const Vector2 &point);

    /**
     * @brief Get the defense area, i.e. the area in front of the goal which is bounded by the penalty line.
     * @param field The field used to determine where the defense area is.
     * @param ourDefenseArea True if our defense area will be returned, false if the opponents defense area will be returned.
     * @param margin The outwards margin in which the defence area will be expanded/shrinked in all directions except for the goal side. A positive value means that it will be
     * expanded, a negative value means that it will be shrinked.
     * @param backMargin The outwards margin at the goal side (boundary side) of the field.
     * @return The area (Polygon) which represents the defense area.
     */
    static Polygon getDefenseArea(const rtt::Field &field, bool ourDefenseArea, double margin, double backMargin);

    /**
     * @brief Get the goal area, i.e. the small area INSIDE the goal.
     * @param field The field used to determine where the goal area is.
     * @param ourGoal True if our goal area has to be returned, false if the opponents goal area has to be returned (by default our goal area will be returned).
     * @param margin The outwards margin in which the defence area will be expanded/shrinked in all directions (except maybe for the goal side). A positive value means that it will
     * be expanded, a negative value means that it will be shrinked (by default it will be neither expanded/shrinked).
     * @param hasBackMargin True if the goal depth has to be increased by the margin (i.e. the goal has to become deeper), false if the goal depth will NOT be increased by the
     * margin (by default the goal depth will NOT be increased by the margin).
     * @return The area (Polygon) that represents the goal area.
     */
    static Polygon getGoalArea(const rtt::Field &field, bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);

    /**
     * @brief Get the entire field area.
     * @param field The field of which the area is determined.
     * @param margin The outwards margin in which the rectangular field area will get expanded/shrinked in all directions. A positive value means that the field area will be
     * expanded, a negative value means that the field area will be shrinked.
     * @return The area (Polygon) that represents the entire field area.
     */
    static Polygon getFieldEdge(const rtt::Field &field, double margin = 0.0);

   private:
    /**
     * @brief Check which part of the goal are blocked by robots, i.e. to which parts of the goal the ball can be shoot over the ground from a given point without hitting any robot
     * from a given team.
     * @param field The field used to determine where the goals are.
     * @param ourGoal True if the blockades on our goal are computed. False if we compute this for the opponents goal.
     * @param point The given point from which it is computed what places of the goal are blocked.
     * @param robots A vector with all robots that could possibly block the goal.
     * @param id The id of the robot which is not considered as blockade. Set this value to -1 if you do not want to exclude a particular robot as blockade (by default we do not
     * exclude a particular robot as blockade).
     * @param ourTeam True if our robots get excluded as blockades. False if all opponents robots get excluded as blockades (by default the opponents robots get excluded as
     * blockades).
     * @return All the parts of the goal that are blocked.
     */
    static std::vector<LineSegment> getBlockadesMappedToGoal(const rtt::Field &field, bool ourGoal, const Vector2 &point, const std::vector<rtt::world::view::RobotView> &robots,
                                                             int id = -1, bool ourTeam = false);

    /**
     * @brief Check whether a given robot really blocks a part of the goal (which is not the case if the robot belongs to a given team or if the robot has a given id) and if so
     * return the given part of the goal that is blocked by this robot.
     * @param ourGoal True if the blockade caused by this robot on our goal is computed. False if we compute this for the opponents goal.
     * @param point The given point from which it is computed what places of the goal is blocked.
     * @param id The id of the robot which is not considered as blockade. Set this value to -1 if you do not want to exclude a particular robot as blockade (by default we do not
     * exclude a particular robot as blockade).
     * @param ourTeam True if our robots get excluded as blockades. False if all opponents robots get excluded as blockades (by default the opponents robots get excluded as
     * blockades).
     * @param robot The robot for which we compute the caused blockade.
     * @param robotRadius The radius of the robot that creates a 'circle' through which the ball is 'considered' not able to pass.
     * @param goalSide The goal side on which the caused blockade is computed.
     * @return std::nullopt in case the robot does not block a part of the goal or if it is not considered a blocking robot. Otherwise it returns the blocked part of the goal.
     */
    static std::optional<LineSegment> robotBlockade(bool ourGoal, const Vector2 &point, int id, bool ourTeam, const rtt::world::view::RobotView robot, const double robotRadius,
                                                    LineSegment goalSide);

    /**
     * @brief Merge overlapping blockade line segments on the goal side into non-overlapping blockade line segments.
     * @param blockades The non-merged possibly overlapping blockade line segments on the goal side.
     * @return Non-overlapping blockade line segments that cover all blocked goal sides.
     */
    static std::vector<LineSegment> mergeBlockades(std::vector<LineSegment> blockades);
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_FIELDCOMPUTATIONS_H
