//
// Created by rolf on 05-11-19.
//

#include "filters/RobotFilter.h"
#include "Scaling.h"

RobotFilter::RobotFilter(const proto::SSL_DetectionRobot &detectionRobot, double detectTime, int cameraID)
    : CameraFilter(detectTime, cameraID), botId{static_cast<int>(detectionRobot.robot_id())} {
    KalmanInit(detectionRobot);
}

/*
 * A short function to sort observations by time.
 */
bool compareObservation(const RobotObservation &a, const RobotObservation &b) { return (a.time < b.time); }
void RobotFilter::update(double time, bool doLastPredict) {
    std::sort(observations.begin(), observations.end(),
              compareObservation);  // First sort the observations in time increasing order
    auto it = observations.begin();
    while (it != observations.end()) {
        auto observation = (*it);
        // the observation is either too old (we already updated the robot) or too new and we don't need it yet.
        if (observation.time < lastUpdateTime) {
            observations.erase(it);
            continue;
        }
        if (observation.time > time) {
            // relevant update, but we don't need the info yet so we skip it.
            ++it;
            continue;
        }
        // We first predict the robot, and then apply the observation to calculate errors/offsets.
        bool cameraSwitched = switchCamera(observation.cameraID, observation.time);
        predict(observation.time, true, cameraSwitched);
        applyObservation(observation);
        observations.erase(it);
    }
    if (doLastPredict) {
        predict(time, false, false);
    }
}

void RobotFilter::KalmanInit(const proto::SSL_DetectionRobot &detectionRobot) {
    // SSL units are in mm, we do everything in SI units.
    double x = mmToM(detectionRobot.x());         // m
    double y = mmToM(detectionRobot.y());         // m
    double angle = detectionRobot.orientation();  // radians [-pi,pi)
    Kalman::Vector startState = Kalman::Vector::Zero();
    startState(0) = x;
    startState(1) = y;
    startState(2) = angle;

    Kalman::Matrix startCov = Kalman::Matrix::Identity();
    // initial noise estimates
    const double startPosNoise = 0.1;
    const double startAngleNoise = 0.1;
    startCov(0, 0) = startPosNoise;    // m noise in x
    startCov(1, 1) = startPosNoise;    // m noise in y
    startCov(2, 2) = startAngleNoise;  // radians
    kalman = std::make_unique<Kalman>(startState, startCov);

    kalman->H = Kalman::MatrixO::Identity();  // Our observations are simply what we see.
}

void RobotFilter::predict(double time, bool permanentUpdate, bool cameraSwitched) {
    double dt = time - lastUpdateTime;
    // forward model:
    kalman->F = Kalman::Matrix::Identity();
    kalman->F(0, 3) = dt;
    kalman->F(1, 4) = dt;
    kalman->F(2, 5) = dt;

    // Set B
    kalman->B = kalman->F;

    // Set u (we have no control input at the moment)
    kalman->u = Kalman::Vector::Zero();

    // Set Q
    const double posNoise = 0.1;
    const double rotNoise = 0.5;
    Kalman::MatrixO G = Kalman::MatrixO::Zero();
    G(0, 0) = dt * posNoise;
    G(0, 3) = 1 * posNoise;
    G(1, 1) = dt * posNoise;
    G(1, 4) = 1 * posNoise;
    G(2, 2) = dt * rotNoise;
    G(2, 5) = 1 * rotNoise;
    // TODO: tune filters
    // We add position errors in case we switch camera because calibration
    if (cameraSwitched) {
        G(0, 0) += 0.02;
        G(1, 1) += 0.02;
        G(2, 2) += 0.04;
    }
    kalman->Q = G.transpose() * G;

    kalman->predict(permanentUpdate);
    if (permanentUpdate) {
        lastUpdateTime = time;
    }
}

/* Updates the kalman filter with the observation.
 * This function assumes you have already predicted until the right time!
 */
void RobotFilter::applyObservation(const RobotObservation &observation) {
    // sanity check
    assert(botId == observation.bot.robot_id());

    Kalman::VectorO obsState = {mmToM(observation.bot.x()), mmToM(observation.bot.y()), 0};
    // We need to do something about the rotation's discontinuities at -pi/pi so it works correctly.
    // We allow the state to go outside of bounds (-PI,PI) in between updates, but then simply make sure the observation difference is correct
    double stateRot = kalman->state()[2];
    double limitedRot = limitAngle(stateRot);
    if (stateRot != limitedRot) {
        kalman->modifyState(2, limitedRot);  // We're adjusting the value of the Kalman Filter here, be careful.
        // We're only doing this so we don't get flips from -inf to inf and loss of double precision at high values.
    }
    double difference = limitAngle(observation.bot.orientation() - stateRot);
    obsState(2) = limitedRot + difference;

    kalman->z = obsState;

    kalman->R = Kalman::MatrixOO::Zero();
    // we put much more trust in observations done by our main camera.
    if (observation.cameraID == mainCamera) {
        // TODO: collect constants somewhere
        const double posVar = 0.006;  // variance in meters
        const double rotVar = 0.01;
        kalman->R(0, 0) = posVar;
        kalman->R(1, 1) = posVar;
        kalman->R(2, 2) = rotVar;
    } else {
        const double posVar = 0.06;  // variance in meters
        const double rotVar = 0.04;
        kalman->R(0, 0) = posVar;
        kalman->R(1, 1) = posVar;
        kalman->R(2, 2) = rotVar;
    }
    kalman->update();
}

double RobotFilter::limitAngle(double angle) const {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

proto::WorldRobot RobotFilter::asWorldRobot() const {
    proto::WorldRobot msg;
    const Kalman::Vector &state = kalman->state();
    msg.set_id(botId);
    msg.mutable_pos()->set_x(state[0]);
    msg.mutable_pos()->set_y(state[1]);
    msg.set_angle(limitAngle(state[2]));  // Need to limit here again (see applyObservation)
    msg.mutable_vel()->set_x(state[3]);
    msg.mutable_vel()->set_y(state[4]);
    msg.set_w(state[5]);
    return msg;
}

void RobotFilter::addObservation(const proto::SSL_DetectionRobot &detectionRobot, double time, int cameraID) {
    observations.emplace_back(RobotObservation(cameraID, time, detectionRobot));
}

double RobotFilter::distanceTo(double x, double y) const {
    const Kalman::Vector &state = kalman->state();
    double dx = state[0] - mmToM(x);
    double dy = state[1] - mmToM(y);
    return sqrt(dx * dx + dy * dy);
}
