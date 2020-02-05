//
// Created by rolf on 17-11-19.
//

#include "filters/BallFilter.h"
#include "Scaling.h"

BallFilter::BallFilter(const proto::SSL_DetectionBall &detectionBall, double detectTime, int cameraID) : CameraFilter(detectTime, cameraID), lastPredictTime{detectTime} {
    KalmanInit(detectionBall);
}
void BallFilter::KalmanInit(const proto::SSL_DetectionBall &detectionBall) {
    // SSL units are in mm, we do everything in SI units.
    double x = mmToM(detectionBall.x());  // m
    double y = mmToM(detectionBall.y());  // m
    Kalman::Vector startState = {x, y, 0, 0};

    Kalman::Matrix startCov = Kalman::Matrix::Identity();
    // initial noise estimates
    const double startPosNoise = 0.05;
    startCov(0, 0) = startPosNoise;  // m noise in x
    startCov(1, 1) = startPosNoise;  // m noise in y

    kalman = std::make_unique<Kalman>(startState, startCov);

    kalman->H = Kalman::MatrixO::Identity();  // Our observations are simply what we see.
}
void BallFilter::applyObservation(const BallObservation &observation) {
    Kalman::VectorO obsPos = {mmToM(observation.ball.x()), mmToM(observation.ball.y())};
    // TODO: do things with the other ball fields (pixel pos, area)
    kalman->z = obsPos;

    // Observations which are not from the main camera are added but are seen as much more noisy
    const double posVar = 0.02;  // variance TODO: tune these 2
    const double posVarOtherCamera = 0.05;
    kalman->R = Kalman::MatrixOO::Zero();
    if (observation.cameraID == mainCamera) {
        kalman->R(0, 0) = posVar;
        kalman->R(1, 1) = posVar;
    } else {
        kalman->R(0, 0) = posVarOtherCamera;
        kalman->R(1, 1) = posVarOtherCamera;
    }
    kalman->update();
}
proto::WorldBall BallFilter::asWorldBall() const {
    proto::WorldBall msg;
    const Kalman::Vector &state = kalman->state();
    msg.mutable_pos()->set_x(state[0]);
    msg.mutable_pos()->set_y(state[1]);
    msg.mutable_vel()->set_x(state[2]);
    msg.mutable_vel()->set_y(state[3]);
    msg.set_visible(ballIsVisible());
    // TODO: add height filter here, and actually set the z, z_vel, area fields
    return msg;
}
double BallFilter::distanceTo(double x, double y) const {
    const Kalman::Vector &state = kalman->state();
    double dx = state[0] - mmToM(x);
    double dy = state[1] - mmToM(y);
    return sqrt(dx * dx + dy * dy);
}
void BallFilter::predict(double time, bool permanentUpdate, bool cameraSwitched) {
    double dt = time - lastUpdateTime;
    // forward model:
    kalman->F = Kalman::Matrix::Identity();
    kalman->F(0, 2) = dt;
    kalman->F(1, 3) = dt;

    // TODO: add 2 stage forward model?
    // Set B
    kalman->B = kalman->F;
    // Set u (we have no control input at the moment)
    kalman->u = Kalman::Vector::Zero();

    // Set Q matrix
    const float posNoise = 0.1;  // TODO: tune
    Kalman::MatrixO G = Kalman::MatrixO::Zero();
    G(0, 0) = dt * posNoise;
    G(0, 2) = 1 * posNoise;
    G(1, 1) = dt * posNoise;
    G(1, 3) = 1 * posNoise;
    if (cameraSwitched) {
        G(0, 0) += 0.05;
        G(1, 1) += 0.05;
    }
    kalman->Q = G.transpose() * G;

    kalman->predict(permanentUpdate);
    lastPredictTime = time;
    if (permanentUpdate) {
        lastUpdateTime = time;
    }
}
bool compareObservation(const BallObservation &a, const BallObservation &b) { return (a.time < b.time); }
void BallFilter::update(double time, bool doLastPredict) {
    std::sort(observations.begin(), observations.end(),
              compareObservation);  // First sort the observations in time increasing order
    auto it = observations.begin();
    while (it != observations.end()) {
        auto observation = (*it);
        // the observation is either too old (we already updated the ball) or too new and we don't need it yet.
        if (observation.time < lastUpdateTime) {
            observations.erase(it);
            continue;
        }
        if (observation.time > time) {
            // relevant update, but we don't need the info yet so we skip it.
            ++it;
            continue;
        }
        // We first predict the ball, and then apply the observation to calculate errors/offsets.
        bool cameraSwitched = switchCamera(observation.cameraID, observation.time);
        predict(observation.time, true, cameraSwitched);
        applyObservation(observation);
        observations.erase(it);
    }
    if (doLastPredict) {
        predict(time, false, false);
    }
}
void BallFilter::addObservation(const proto::SSL_DetectionBall &detectionBall, double time, int cameraID) {
    observations.emplace_back(BallObservation(cameraID, time, detectionBall));
}
bool BallFilter::ballIsVisible() const {
    // If we extrapolated the ball for longer than 0.05 seconds we mark it not visible
    return (lastPredictTime - lastUpdateTime) < 0.05;
}
