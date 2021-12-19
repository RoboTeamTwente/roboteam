#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H

#include <roboteam_proto/World.pb.h>
#include <roboteam_proto/WorldRobot.pb.h>
#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>

#include "observer/filters/vision/ball/BallFilter.h"
#include "observer/filters/vision/robot/RobotFilter.h"
#include "DetectionFrame.h"
#include "observer/parameters/RobotParameterDatabase.h"

/**
 * @author Rolf van der Hulst
 * @date November 2019
 * @brief class that tracks a world. The function of this class is to determine when to create and delete filters,
 * and to pass the relevant incoming information to the relevant filter(s)
 */
class WorldFilter {
 public:
  WorldFilter();

  void process(const std::vector<proto::SSL_DetectionFrame> &frames);

  [[nodiscard]] proto::World getWorldPrediction(const Time& time) const;

  /**
   * Updates the cameras which the worldFilter uses for calculations, and the field geometry which is used for detecting
   * wall collisions.
   * @param geometry to grab the cameras from
   */
  void updateGeometry(const proto::SSL_GeometryData& geometry);

  void updateRobotParameters(const TwoTeamRobotParameters& robotInfo);

 private:
  typedef std::map<RobotID, std::vector<RobotFilter>> robotMap;
  robotMap blue;
  robotMap yellow;
  std::vector<BallFilter> balls;

  RobotParameters blueParams;
  RobotParameters yellowParams;

  std::map<int,Time> lastCaptureTimes;

  constexpr static int MAX_ROBOTFILTERS = 5; //maximum number of object filters for one robot ID
  constexpr static int MAX_BALLFILTERS = 8; //maximum number of object filters for one ball ID

  void processFrame(const DetectionFrame& frame);
  void processRobots(const DetectionFrame& frame, bool blueBots);
  void processBalls(const DetectionFrame& frame);
  [[nodiscard]] std::vector<FilteredRobot> getHealthiestRobotsMerged(bool blueBots, Time time)  const;
  [[nodiscard]] std::vector<FilteredRobot> oneCameraHealthyRobots(bool blueBots, int camera_id, Time time) const;
  void addRobotPredictionsToMessage(proto::World& world, Time time) const;
  void addBallPredictionsToMessage(proto::World& world, Time time) const;

  //do they care, although these method are static, they DO modify the current object as they have a robotMap&
  static void predictRobots(const DetectionFrame &frame, robotMap &robots);
  static void updateRobots(robotMap &robots, const std::vector<RobotObservation> &detectedRobots);
  static void updateRobotsNotSeen(const DetectionFrame &frame,robotMap &robots);
};

#endif  // ROBOTEAM_WORLD_KALMANFILTER_H
