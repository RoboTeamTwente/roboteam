#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H

#include <proto/World.pb.h>
#include <proto/WorldRobot.pb.h>
#include <proto/messages_robocup_ssl_geometry.pb.h>

#include "DetectionFrame.h"
#include "RobotFeedbackFilter.h"
#include "observer/filters/vision/ball/BallFilter.h"
#include "observer/filters/vision/robot/RobotFilter.h"
#include "observer/parameters/RobotParameterDatabase.h"

/**
 * @author Rolf van der Hulst
 * @date November 2019
 * @brief class that tracks a world. This class is responsible for tracking and updating quickly-changing information,
 * such as the positions and velocities of the robots.
 *
 * The primary function of this class is to determine when to create and delete filters,
 * and to divide the incoming information over the relevant filters
 */
class WorldFilter {
   public:
    WorldFilter();

    void process(const std::vector<proto::SSL_DetectionFrame>& frames, const std::vector<rtt::RobotsFeedback>& feedback);

    [[nodiscard]] proto::World getWorldPrediction(const Time& time) const;

    void updateRobotParameters(const TwoTeamRobotParameters& robotInfo);

   private:
    typedef std::map<RobotID, std::vector<RobotFilter>> robotMap;
    robotMap blue;
    robotMap yellow;
    std::vector<BallFilter> balls;

    RobotParameters blueParams;
    RobotParameters yellowParams;

    std::map<int, Time> lastCaptureTimes;

    constexpr static int MAX_ROBOTFILTERS = 5;  // maximum number of object filters for one robot ID
    constexpr static int MAX_BALLFILTERS = 8;   // maximum number of object filters for one ball ID

    RobotFeedbackFilter feedbackFilter;
    void processFrame(const DetectionFrame& frame);
    void processRobots(const DetectionFrame& frame, bool blueBots);
    void processBalls(const DetectionFrame& frame);
    [[nodiscard]] std::vector<FilteredRobot> getHealthiestRobotsMerged(bool blueBots, Time time) const;
    [[nodiscard]] std::vector<FilteredRobot> oneCameraHealthyRobots(bool blueBots, int camera_id, Time time) const;
    void addRobotPredictionsToMessage(proto::World& world, Time time) const;
    void addBallPredictionsToMessage(proto::World& world, Time time) const;

    // take care, although these method are static, they typically DO modify the current object as they have a robotMap reference
    static void predictRobots(const DetectionFrame& frame, robotMap& robots);
    static void updateRobots(robotMap& robots, const std::vector<RobotObservation>& detectedRobots);
    static void updateRobotsNotSeen(const DetectionFrame& frame, robotMap& robots);
};

#endif  // ROBOTEAM_WORLD_KALMANFILTER_H
