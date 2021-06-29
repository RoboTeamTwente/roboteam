//
// Created by kjhertenberg on 13-5-19.
//

#include "filters/vision/WorldFilter.h"

WorldFilter::WorldFilter() {
}

void WorldFilter::updateGeometry(const proto::SSL_GeometryData &geometry) {
  //TODO: fix
}

proto::World WorldFilter::getWorldPrediction(const Time &time) const {
  //TODO: split up in functions for robot and ball
  proto::World world;
  addRobotPredictionsToMessage(world,time);
  world.set_time(time.asNanoSeconds());

  return world;
}

void WorldFilter::process(const std::vector<proto::SSL_DetectionFrame> &frames) {
  std::vector<DetectionFrame> detectionFrames;
  for (const auto &protoFrame: frames) {
    detectionFrames.emplace_back(DetectionFrame(protoFrame));
  }
  //Sort by time
  std::sort(detectionFrames.begin(), detectionFrames.end(),
            [](const DetectionFrame &lhs, const DetectionFrame &rhs) { return lhs.timeCaptured < rhs.timeCaptured; });
  for (auto &frame : detectionFrames) {
    auto cameraTime = lastCaptureTimes.find(frame.cameraID);
    frame.dt = cameraTime == lastCaptureTimes.end() ? 0.0 : (cameraTime->second -
        lastCaptureTimes[frame.cameraID]).asSeconds();
    //TODO: make a realtime option
    //TODO: remove any frames with captures times which differ more than a second from the current time
  }
  //Remove frames which are too late. For now we do this, because it's quite hard to go back in time and reconstruct the state of the entire visionFilter

  //This can also be caused by other teams running e.g. their simulators internally and accidentally broadcasting onto the network
  detectionFrames.erase(std::remove_if(detectionFrames.begin(), detectionFrames.end(),
                                       [](const DetectionFrame &frame) { return frame.dt < 0.0; }),detectionFrames.end());
  for (const auto &frame : detectionFrames) {
    processFrame(frame);
  }
}


void WorldFilter::processRobots(const DetectionFrame &frame, bool blueBots) {
  robotMap &robots = blueBots ? blue : yellow;
  const std::vector<RobotObservation> &detectedRobots = blueBots ? frame.blue : frame.yellow;

  predictRobots(frame, robots);
  updateRobots(robots, detectedRobots);
  updateRobotsNotSeen(frame, robots);
}

void WorldFilter::updateRobotsNotSeen(const DetectionFrame &frame, robotMap &robots) {
  for (auto &oneIDFilters : robots) {
    std::vector<RobotFilter> &filters = oneIDFilters.second;
    auto it = filters.begin();
    while (it != filters.end()) {
      if (it->processNotSeen(frame.cameraID, frame.timeCaptured)) { //updates the relevant object filter. If the filter is redundant, we can remove it
        it = filters.erase(it);
      } else {
        it++;
      }
    }
  }
}

void WorldFilter::updateRobots(robotMap &robots,
                               const std::vector<RobotObservation> &detectedRobots) {
  //add detected robots to existing filter(s) or create a new filter if no filter accepts the robot
  for (const auto &detectedRobot : detectedRobots) {
    if (!detectedRobot.robot.robot_id.isValid()) {
      continue;
    }
    std::vector<RobotFilter> &oneIDFilters = robots[detectedRobot.robot.robot_id];
    bool accepted = false;
    for (RobotFilter &filter : oneIDFilters) {
      accepted |= filter.processDetection(detectedRobot);
    }
    if (!accepted && oneIDFilters.size() < MAX_ROBOTFILTERS) {
      oneIDFilters.emplace_back(RobotFilter(detectedRobot));
    }
  }
}

void WorldFilter::predictRobots(const DetectionFrame &frame, robotMap &robots) {
  for (auto &oneIDFilters : robots) {
    for (auto &filter : oneIDFilters.second) {
      filter.predictCam(frame.cameraID, frame.timeCaptured);
    }
  }
}

void WorldFilter::processFrame(const DetectionFrame &frame) {
  processRobots(frame, true);
  processRobots(frame, false);

}



void WorldFilter::updateRobotParameters(const TwoTeamRobotParameters& parameters) {
    blueParams = parameters.blueParameters;
    yellowParams = parameters.yellowParameters;
}



std::vector<FilteredRobot> WorldFilter::getHealthiestRobotsMerged(bool blueBots, Time time) const {
  std::vector<FilteredRobot> robots;
  const robotMap &map = blueBots ? blue : yellow;
  for (const auto &oneIDFilters : map) {
    if (oneIDFilters.second.empty()) {
      continue;;
    }
    double maxHealth = -std::numeric_limits<double>::infinity();
    auto bestFilter = oneIDFilters.second.begin();
    for (auto robotFilter = oneIDFilters.second.begin();
         robotFilter != oneIDFilters.second.end(); ++robotFilter) {
      double health = robotFilter->getHealth();
      if (health > maxHealth) {
        maxHealth = health;
        bestFilter = robotFilter;
      }
    }
    robots.push_back(bestFilter->mergeRobots(time));
  }
  return robots;
}

std::vector<FilteredRobot> WorldFilter::oneCameraHealthyRobots(bool blueBots, int camera_id, Time time) const {
  std::vector<FilteredRobot> robots;
  const robotMap &map = blueBots ? blue : yellow;
  for (const auto &oneIDFilters : map) {
    if (oneIDFilters.second.empty()) {
      continue;;
    }
    double maxHealth = -std::numeric_limits<double>::infinity();
    auto bestFilter = oneIDFilters.second.begin();
    for (auto robotFilter = oneIDFilters.second.begin();
         robotFilter != oneIDFilters.second.end(); ++robotFilter) {
      double health = robotFilter->getHealth();
      if (health > maxHealth) {
        maxHealth = health;
        bestFilter = robotFilter;
      }
    }
    auto robot = bestFilter->getRobot(camera_id, time);
    if (robot) {
      robots.push_back(robot.value());
    }
  }
  return robots;
}
void WorldFilter::addRobotPredictionsToMessage(proto::World &world, Time time) const{
  std::vector<FilteredRobot> blueRobots = getHealthiestRobotsMerged(true, time);
  for (const auto &blueBot : blueRobots) {
    world.mutable_blue()->Add()->CopyFrom(blueBot.asWorldRobot());
  }
  std::vector<FilteredRobot> yellowRobots = getHealthiestRobotsMerged(false, time);
  for (const auto &yellowBot : yellowRobots) {
    world.mutable_yellow()->Add()->CopyFrom(yellowBot.asWorldRobot());
  }
}



