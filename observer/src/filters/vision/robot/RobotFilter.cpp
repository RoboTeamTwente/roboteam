//
// Created by rolf on 05-11-19.
//

//
// Created by rolf on 05-08-20.
//

#include "filters/vision/robot/RobotFilter.h"


bool RobotFilter::processDetection(const RobotObservation &observation) {
  auto cameraFilter = cameraFilters.find(observation.cameraID);
  if (cameraFilter != cameraFilters.end()) {
    bool accept = cameraFilter->second.acceptObservation(observation);
    if (accept) {
      cameraFilter->second.update(observation);
    }
    return accept;
  } else {
    assert(!cameraFilters.empty());

    bool accept = true;
    for (const auto &filter : cameraFilters) {
      accept &= filter.second.acceptObservation(observation);
    }
    if (accept) {
      //We can initialize this new filter with information from the other filters, by giving it the initial speed the others detected
      //TODO: make this a weighted average (using e.g. filter age / health?)
      RobotVel velocity = RobotVel();
      for (const auto &filter : cameraFilters) {
        velocity += filter.second.velocityEstimate(observation.timeCaptured);
      }
      velocity /= double(cameraFilters.size());
      cameraFilters.insert(std::make_pair(observation.cameraID, CameraRobotFilter(observation,velocity)));
    }
    return accept;
  }
}

RobotFilter::RobotFilter(const RobotObservation &observation) :
    id{observation.robot},
    cameraFilters{std::make_pair(observation.cameraID, CameraRobotFilter(observation))} {
}

bool RobotFilter::processNotSeen(int cameraID, const Time &time) {
  auto cameraFilter = cameraFilters.find(cameraID);
  if (cameraFilter == cameraFilters.end()) {
    return false; //if the relevant camera does not exist, we do not need to remove it
  }
  if (!cameraFilter->second.justUpdated()) {
    bool removeFilter = cameraFilter->second.updateRobotNotSeen(time);
    if (removeFilter) {
      cameraFilters.erase(cameraFilter);
    }
  }
  return cameraFilters.empty();
}

void RobotFilter::predictCam(const int &cameraID, const Time &untilTime) {
  auto cameraFilter = cameraFilters.find(cameraID);
  if (cameraFilter != cameraFilters.end()) {
    cameraFilter->second.predict(untilTime);
  }
}
double RobotFilter::getHealth() const {
  double maxHealth = 0.0;
  for (const auto &filter : cameraFilters) {
    maxHealth = fmax(filter.second.getHealth(), maxHealth);
  }
  return maxHealth;
}
FilteredRobot RobotFilter::mergeRobots(const Time &time) const {
  double mergeFactor = 1.5;
  Eigen::Vector2d vel(0, 0);
  Eigen::Vector2d pos(0, 0);
  double angle = 0;
  double angularVel = 0;
  double totalPosUncertainty = 0;
  double totalVelUncertainty = 0;
  double totalAngleUncertainty = 0;
  double totalAngleVelUncertainty = 0;
  //We cannot take averages of angular coordinates easily, so we take the averages of the offsets (this works)
  double angleOffset = cameraFilters.begin()->second.estimate(time).position.angle;

  //TODO: consider more fancy  merging using covariance matrices, which is not reliant on the health information
  for (const auto &filter : cameraFilters) {
    FilteredRobot robot = filter.second.estimate(time);
    //Use the filter health and uncertainties for a weighted average of observations
    double weight = 100.0/robot.health; //TODO: call MAXIMUM here from object
    double posWeight = pow(robot.posUncertainty*weight, - mergeFactor);
    double velWeight = pow(robot.velocityUncertainty*weight, - mergeFactor);
    double angleWeight = pow(robot.angleUncertainty*weight,-mergeFactor);
    double angleVelWeight = pow(robot.angularVelUncertainty*weight,-mergeFactor);

    pos += robot.position.position*posWeight;
    vel += robot.velocity.velocity*velWeight;
    angle += double(robot.position.angle - rtt::Angle(angleOffset)) * angleWeight;
    angularVel += robot.velocity.angularVelocity*angleVelWeight;

    totalPosUncertainty += posWeight;
    totalVelUncertainty += velWeight;
    totalAngleUncertainty += angleWeight;
    totalAngleVelUncertainty += angleVelWeight;

  }
  pos /= totalPosUncertainty;
  vel /= totalVelUncertainty;
  angle /= totalAngleUncertainty;
  angularVel /= totalAngleVelUncertainty;
  FilteredRobot result(id, RobotPos(pos,angle),RobotVel(vel,angularVel),
                       -1,-1,-1,-1,-1); //TODO define health and merged uncertainties?

  return result;
}

std::optional<FilteredRobot> RobotFilter::getRobot(int cameraID, Time time) const {
  auto cameraFilter = cameraFilters.find(cameraID);
  if(cameraFilter != cameraFilters.end()){
    return cameraFilter->second.estimate(time);
  }else{
    return std::nullopt;
  }
}



