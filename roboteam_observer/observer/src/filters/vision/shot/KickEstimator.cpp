#include "filters/shot/KickEstimator.h"

#include <Eigen/Dense>

KickEstimator::KickEstimator(const ShotEvent& shotEvent, const BallParameters& ballParameters) : shotEvent(shotEvent), ballParameters(ballParameters) {
    for (const auto& ball : shotEvent.ballsSinceShot) {
        if (ball.currentObservation.has_value()) {
            ballsSinceShot.push_back(ball.currentObservation.value());
        }
    }
}

void KickEstimator::addFilteredBall(const BallObservation& newBall) {
    ballsSinceShot.push_back(newBall);
    if (ballsSinceShot.size() > 50) {
        ballsSinceShot.erase(ballsSinceShot.begin() + pruneIndex);
        pruneIndex++;
        if (pruneIndex > 40) {
            pruneIndex = 1;
        }
    }
    // TODO: MAYBE IMPLEMENT BALL WITH INITIAL SPIN AS WELL?
    std::pair<Eigen::Vector3d, Eigen::Vector3d> slidingPosVel = slidingBall();
    // Set estimate if there is non yet
    if (bestShotPos.norm() == 0 && bestShotVel.norm() == 0) {
        bestShotPos = slidingPosVel.first;
        bestShotVel = slidingPosVel.second;
    }
    std::pair<Eigen::Vector3d, Eigen::Vector3d> nonLinPosVel = noSpinBall();
    double slidingDistance = getAverageDistance(slidingPosVel.first, slidingPosVel.second, Eigen::Vector2d(0, 0));
    double noSpinDistance = getAverageDistance(nonLinPosVel.first, nonLinPosVel.second, Eigen::Vector2d(0, 0));
    if (slidingDistance < noSpinDistance) {
        bestShotPos = slidingPosVel.first;
        bestShotVel = slidingPosVel.second;
    } else {
        bestShotPos = nonLinPosVel.first;
        bestShotVel = nonLinPosVel.second;
    }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> KickEstimator::noSpinBall() {
    auto dir = getKickDir();
    auto objectiveFunction = [&](const dlib::matrix<double, 0, 1>& params) -> double {
        Eigen::Vector3d shotPos(params(0), params(1), 0);
        auto shotVelInt = dir * params(2);
        Eigen::Vector3d shotVel(shotVelInt.x(), shotVelInt.y(), 0);
        Eigen::Vector2d shotSpin(0, 0);
        return getAverageDistance(shotPos, shotVel, shotSpin);
    };

    // Initial guess for the parameters
    dlib::matrix<double, 0, 1> initialParams(3);
    initialParams(0) = bestShotPos.x();
    initialParams(1) = bestShotPos.y();
    initialParams(2) = bestShotVel.norm();

    dlib::matrix<double, 0, 1> x_lower(3), x_upper(3);
    x_lower(0) = bestShotPos.x() - 1;
    x_lower(1) = bestShotPos.y() - 1;
    x_lower(2) = bestShotVel.norm() - 1;
    x_upper(0) = bestShotPos.x() + 1;
    x_upper(1) = bestShotPos.y() + 1;
    x_upper(2) = bestShotVel.norm() + 1;

    for (long i = 0; i < initialParams.size(); ++i) {
        if (initialParams(i) < x_lower(i)) {
            initialParams(i) = x_lower(i);
        } else if (initialParams(i) > x_upper(i)) {
            initialParams(i) = x_upper(i);
        }
    }
    try {
        dlib::find_min_bobyqa(objectiveFunction, initialParams, 7, x_lower, x_upper, 0.2, 1e-3, 50);
    } catch (const dlib::bobyqa_failure& e) {
    }

    auto shotPos = Eigen::Vector3d(initialParams(0), initialParams(1), 0);
    auto shotVel = Eigen::Vector3d(dir.x() * initialParams(2), dir.y() * initialParams(2), 0);
    return std::make_pair(shotPos, shotVel);
}

Eigen::Vector2d KickEstimator::getKickDir() {
    int numRecords = ballsSinceShot.size();
    std::vector<Eigen::Vector2d> groundPos;
    for (int i = 0; i < numRecords; i++) {
        groundPos.push_back(ballsSinceShot[i].position);
    }

    double sumX = 0.0, sumY = 0.0, sumX2 = 0.0, sumXY = 0.0;
    for (int i = 0; i < numRecords; ++i) {
        double x = ballsSinceShot[i].position.x();
        double y = ballsSinceShot[i].position.y();
        sumX += x;
        sumY += y;
        sumX2 += x * x;
        sumXY += x * y;
    }

    // Calculate the slope (m) and intercept (b)
    double m = (numRecords * sumXY - sumX * sumY) / (numRecords * sumX2 - sumX * sumX);
    // double b = (sumY - m * sumX) / numRecords;

    Eigen::Vector2d dir(1, m);
    dir.normalize();
    return dir;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> KickEstimator::slidingBall() {
    int numRecords = ballsSinceShot.size();
    auto tZero = ballsSinceShot[0].timeCaptured;
    double acc = ballParameters.getAccSlide();
    auto dir = getKickDir();

    Eigen::MatrixXd matA(numRecords * 2, 3);
    Eigen::VectorXd b(numRecords * 2);

    for (int i = 0; i < numRecords; i++) {
        const auto& ballRecord = ballsSinceShot[i];

        auto g = ballRecord.position;
        double t = (ballRecord.timeCaptured - tZero).asSeconds();

        matA.row(i * 2) << 1, 0, dir.x() * t;
        matA.row(i * 2 + 1) << 0, 1, dir.y() * t;

        b(i * 2) = g.x() - (0.5 * dir.x() * t * t * acc);
        b(i * 2 + 1) = g.y() - (0.5 * dir.y() * t * t * acc);
    }

    Eigen::VectorXd x;
    try {
        x = matA.colPivHouseholderQr().solve(b);
    } catch (const std::exception& e) {
        return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    }
    dir *= x(2) / dir.norm();
    auto shotPos = Eigen::Vector3d(x(0), x(1), 0);
    auto shotVel = Eigen::Vector3d(dir.x(), dir.y(), 0);
    return std::make_pair(shotPos, shotVel);
}

double KickEstimator::getAverageDistance(Eigen::Vector3d shotPos, Eigen::Vector3d shotVel, Eigen::Vector2d shotSpin) {
    KickTrajectory kickTrajectory = KickTrajectory(shotPos, shotVel, shotSpin, ballParameters);
    auto tZero = ballsSinceShot[0].timeCaptured;
    double totalDistance = 0;
    for (std::size_t i = 0; i < ballsSinceShot.size(); i++) {
        double time = (ballsSinceShot[i].timeCaptured - tZero).asSeconds();
        auto trajState = kickTrajectory.getPositionAtTime(time);
        auto pos2d = trajState.pos.head<2>();
        double distance = (pos2d - ballsSinceShot[i].position).norm();
        totalDistance += distance;
    }
    return totalDistance / ballsSinceShot.size();
}

double KickEstimator::getAverageDistance() { return getAverageDistance(bestShotPos, bestShotVel, Eigen::Vector2d(0, 0)); }