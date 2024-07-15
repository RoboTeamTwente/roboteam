#ifndef RTT_PASSCOMPUTATIONS_H
#define RTT_PASSCOMPUTATIONS_H

#include <roboteam_utils/LineSegment.h>
#include <stp/computations/InterceptionComputations.h>

#include <roboteam_utils/Field.hpp>

#include "roboteam_utils/Grid.h"
#include "world/World.hpp"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp {

/**
 * @brief Struct to hold relevant information for passing
 */
struct PassInfo {
    int keeperId = -1;
    int passerId = -1;
    int receiverId = -1;
    Vector2 passLocation;                      // The location where the ball will be passed from, towards the receiver
    Vector2 receiverLocation = Vector2(6, 0);  // The location of the receiver, who will receive the ball from the passer
    uint8_t passScore = 0;
};

}  // namespace rtt::ai::stp
namespace rtt::ai::stp::computations {

/**
 * @brief Class with computations about passing
 */
class PassComputations {
   public:
    /**
     * @brief Calculates which robot should pass where, and which robot should receive it
     * @param profile the profile to be used when scoring the pass location
     * @param world the current world state
     * @param field the current field
     * @param keeperMustPass indicate whether the keeper must pass
     * @return a PassInfo struct which contains the relevant information needed to complete the pass
     */
    static PassInfo calculatePass(gen::ScoreProfile profile, const world::World* world, const Field& field, bool keeperMustPass = false);

   private:
    /**
     * @brief Gets the grid of points containing all possible pass locations
     * @param field the current field
     * @return a Grid class containing a vector of vectors, which in turn contain all possible pass locations
     */
    static Grid getPassGrid(const Field& field);

    /**
     * @brief Indicates whether the given point 1) a valid point to pass to in terms of ssl-rules and 2) whether it is feasible ot pass there
     * @param point the point to check for validity
     * @param possibleReceiverLocations the locations of all robots that could receive the ball
     * @param possibleReceiverVelocities the velocities of all robots that could receive the ball
     * @param possiblePassLocations the locations of all robots that could pass the ball
     * @param passLocation the location where the ball will be passed from
     * @param passerLocation the location of the robot that will pass the ball
     * @param passerVelocity the velocity of the robot that will pass the ball
     * @param passerId the id of the robot that will pass the ball
     * @param field the current field
     * @param world the current world
     * @return bool indicating whether this point is (likely) possible to pass to
     */

    static bool pointIsValidReceiverLocation(Vector2 point, const std::vector<Vector2>& possibleReceiverLocations, const std::vector<Vector2>& possibleReceiverVelocities,
                                             const std::vector<int>& possibleReceiverIds, Vector2 passLocation, Vector2 passerLocation, Vector2 passerVelocity, int passerId,
                                             const Field& field, const world::World* world);

    /**
     * @brief Approximate the time it takes a robot to reach a point
     * @param robotPosition current position of robot
     * @param robotVelocity current velocity of robot
     * @param robotId id of robot
     * @param targetPosition position to calculate travel time to
     * @return approximated time to reach target from position
     */
    static double calculateRobotTravelTime(Vector2 robotPosition, Vector2 robotVelocity, int robotId, Vector2 targetPosition);

    /**
     * Approximate the time it takes the ball to reach a point
     * @param passLocation the location where the ball will be passed from
     * @param passerLocation current location of the passer
     * @param passerId id of the passer
     * @param robotVelocity current velocity of robot
     * @param targetPosition position to calculate travel time to
     * @return approximated time for ball to reach target position
     */
    static double calculateBallTravelTime(Vector2 passLocation, Vector2 passerLocation, int passerId, Vector2 passerVelocity, Vector2 targetPosition);
};
}  // namespace rtt::ai::stp::computations
#endif  // RTT_PASSCOMPUTATIONS_H
