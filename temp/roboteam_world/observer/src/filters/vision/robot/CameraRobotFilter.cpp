//
// Created by rolf on 23-06-21.
//

#include "filters/vision/robot/CameraRobotFilter.h"



CameraRobotFilter::CameraRobotFilter(const RobotObservation& observation, RobotVel velocityEstimate) :
    CameraObjectFilter(0.2, 1 / 60.0, 10, 3, observation.timeCaptured),
    robot{observation.robot},
    cameraID{observation.cameraID}{
  //Initialize position filter
  //TODO: initialize from other camera

  constexpr double ROBOT_POSITION_INITIAL_COV = 0.1; //[m] Uncertainty in initial robot position
  constexpr double ROBOT_VELOCITY_INITIAL_COV = 4.0; //[m/s] Uncertainty in initial robot velocity (which is 0 for new robots)
  constexpr double ROBOT_POSITION_MEASUREMENT_ERROR = 0.005; //[m] Estimated average position uncertainty in robot detections
  constexpr double ROBOT_POSITION_MODEL_ERROR = 4.0 ; // [m/s^2] Assumed white noise in acceleration of a robot for process error

  constexpr double ROBOT_ANGLE_INITIAL_COV = 0.20;// [rad] 11.5 degrees roughly
  constexpr double ROBOT_ANGULAR_VEL_INITIAL_COV = 5.0; //[rad/s] Uncertainty in initial w
  constexpr double ROBOT_ANGLE_MEASUREMENT_ERROR = 0.02 ; //[rad] 1.1 degrees roughly //TODO measure in practice/tune
  constexpr double ROBOT_ANGLE_MODEL_ERROR =  4.0;//[rad/s^2] Assumed white noise in angular acceleration of a robot

  Eigen::Matrix4d initialPosCov = Eigen::Matrix4d::Zero();
  initialPosCov(0,0) = ROBOT_POSITION_INITIAL_COV;
  initialPosCov(1,1) = ROBOT_POSITION_INITIAL_COV;
  initialPosCov(2,2) = ROBOT_VELOCITY_INITIAL_COV;
  initialPosCov(3,3) = ROBOT_VELOCITY_INITIAL_COV;

  Eigen::Vector4d initialPos ={observation.position.x(),observation.position.y(),velocityEstimate.velocity.x(),velocityEstimate.velocity.y()};
  positionFilter = PosVelFilter2D(initialPos,initialPosCov,ROBOT_POSITION_MODEL_ERROR,ROBOT_POSITION_MEASUREMENT_ERROR,observation.timeCaptured);

  Eigen::Vector2d initialAngle = {observation.orientation,velocityEstimate.angularVelocity};
  Eigen::Matrix2d initialAngleCov = Eigen::Matrix2d::Zero();
  initialAngleCov(0,0) = ROBOT_ANGLE_INITIAL_COV;
  initialAngleCov(1,1) = ROBOT_ANGULAR_VEL_INITIAL_COV;
  angleFilter = RobotOrientationFilter(initialAngle,initialAngleCov,ROBOT_ANGLE_MODEL_ERROR,ROBOT_ANGLE_MEASUREMENT_ERROR,observation.timeCaptured);

  previousPosition = RobotPos(observation.position,rtt::Angle(observation.orientation));

  previousTime = Time(observation.timeCaptured);

  just_updated = true;
}

void CameraRobotFilter::predict(Time time) {
  updatePreviousInfo();
  positionFilter.predict(time);
  angleFilter.predict(time);
  just_updated = false;
}

void CameraRobotFilter::update(const RobotObservation &observation) {
  assert(observation.robot == robot); //sanity check
  assert(observation.cameraID == cameraID);
  //Update position kalman filter
  positionFilter.update(observation.position);
  //Update angle kalman filter
  angleFilter.update(observation.orientation);
  //update object seen settings
  objectSeen(observation.timeCaptured);
  just_updated = true;
}

bool CameraRobotFilter::updateRobotNotSeen(const Time &time) {
  objectInvisible(time);
  return getHealth() <= 0.0 && consecutiveFramesNotSeen() > 3; // We remove a robot if it's health is zero and it has not been seen for 3 frames
}


bool CameraRobotFilter::justUpdated() const {
  return just_updated;
}

FilteredRobot CameraRobotFilter::estimate(const Time &time) const {
  FilteredRobot bot(
      robot,
      RobotPos(positionFilter.getPositionEstimate(time),angleFilter.getPositionEstimate(time)),
      RobotVel(positionFilter.getVelocity(),angleFilter.getVelocity()),
      getHealth(),
      positionFilter.getPositionUncertainty().norm(),
      positionFilter.getVelocityUncertainty().norm(),
      angleFilter.getPositionUncertainty(),
      angleFilter.getVelocityUncertainty()
      );
  return bot;
}


bool CameraRobotFilter::acceptObservation(const RobotObservation& observation) const{
  double angleDif = abs(rtt::Angle(angleFilter.getPosition()-observation.orientation));
  double posDifSq = (observation.position-positionFilter.getPosition()).squaredNorm();
  return posDifSq<0.4*0.4 && angleDif < M_PI_2;
}

RobotVel CameraRobotFilter::velocityEstimate(const Time &time) const {
  Eigen::Vector2d vel = positionFilter.getVelocity();
  double angVel = angleFilter.getVelocity();
  return RobotVel(vel,angVel);
}

void CameraRobotFilter::updatePreviousInfo() {
  previousPosition = RobotPos(positionFilter.getPosition(),angleFilter.getPosition());

  previousTime = positionFilter.lastUpdated();
}