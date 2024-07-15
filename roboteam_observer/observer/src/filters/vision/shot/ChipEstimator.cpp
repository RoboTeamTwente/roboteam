#include "filters/shot/ChipEstimator.h"

#include <dlib/optimization.h>

#include <Eigen/Dense>

ChipEstimator::ChipEstimator(const ShotEvent& shotEvent, const BallParameters& ballParameters, const CameraMap& cameraMap)
    : shotEvent(shotEvent), ballParameters(ballParameters), cameraMap(cameraMap) {
    for (const auto& ball : shotEvent.ballsSinceShot) {
        if (ball.currentObservation.has_value()) {
            if ((ball.currentObservation.value().position - shotEvent.ballPosition).norm() < 0.05) {
                continue;
            }
            ballsSinceShot.push_back(ball.currentObservation.value());
        }
    }
    bestShotPos = Eigen::Vector3d(shotEvent.ballPosition.x(), shotEvent.ballPosition.y(), 0);
}

void ChipEstimator::addFilteredBall(const BallObservation& newBall) {
    ballsSinceShot.push_back(newBall);
    if (ballsSinceShot.size() > 50) {
        ballsSinceShot.erase(ballsSinceShot.begin() + pruneIndex);
        pruneIndex++;
        if (pruneIndex > 40) {
            pruneIndex = 1;
        }
    }
    // only predict based on the first hop, bit janky but it _might_ work
    if (doFirstHop && ballsSinceShot.size() > 8) {
        auto tOff = 0.05;
        auto inc = tOff / 2;
        while (inc > 1e-3) {
            auto resNeg = noSpinBall(tOff - 1e-5);
            auto resPos = noSpinBall(tOff + 1e-5);
            if (resNeg.second < resPos.second) {
                tOff -= inc;
            } else {
                tOff += inc;
            }
            inc /= 2;
        }
        auto res = noSpinBall(tOff);
        bestShotVel = res.first;
        bestTOff = tOff;

        auto tFly = 2 * bestShotVel.z() / 9.81;
        if ((newBall.timeCaptured - ballsSinceShot[0].timeCaptured).asSeconds() > tFly) {
            doFirstHop = false;
            std::cout << "\033[33mFirst hop done\033[0m" << std::endl;
        }
    }
}

std::pair<Eigen::Vector3d, double> ChipEstimator::noSpinBall(double tOff) {
    int numRecords = ballsSinceShot.size();
    auto tZero = ballsSinceShot[0].timeCaptured;
    double pz = 0.02;

    Eigen::MatrixXd matA(numRecords * 2, 3);
    Eigen::VectorXd b(numRecords * 2);

    for (int i = 0; i < numRecords; i++) {
        const auto& ballRecord = ballsSinceShot[i];

        auto g = ballRecord.position;
        double t = (ballRecord.timeCaptured - tZero).asSeconds() + tOff;
        auto f = cameraMap[ballRecord.cameraID].position();

        matA.row(i * 2) << f.z() * t, 0, (g.x() - f.x()) * t;
        matA.row((i * 2) + 1) << 0, f.z() * t, (g.y() - f.y()) * t;

        b(i * 2) = ((0.5 * 9.81 * t * t * (g.x() - f.x())) + (g.x() * f.z())) - (bestShotPos.x() * f.z()) - ((g.x() - f.x()) * pz);
        b((i * 2) + 1) = ((0.5 * 9.81 * t * t * (g.y() - f.y())) + (g.y() * f.z())) - (bestShotPos.y() * f.z()) - ((g.y() - f.y()) * pz);
    }

    Eigen::VectorXd x;
    try {
        x = matA.colPivHouseholderQr().solve(b);
    } catch (const std::exception& e) {
    }
    auto bestVel = Eigen::Vector3d(x(0), x(1), x(2));
    auto l1Error = (matA * x - b).lpNorm<1>();
    return std::make_pair(bestVel, l1Error);
}

double ChipEstimator::getAverageDistance(Eigen::Vector3d shotPos, Eigen::Vector3d shotVel, Eigen::Vector2d shotSpin) {
    ChipTrajectory chipTrajectory = ChipTrajectory(shotPos, shotVel, shotSpin, ballParameters);

    auto tZero = ballsSinceShot[0].timeCaptured;
    double totalDistance = 0;

    for (std::size_t i = 0; i < ballsSinceShot.size(); i++) {
        double time = (ballsSinceShot[i].timeCaptured - tZero).asSeconds() + bestTOff;
        auto realPos = chipTrajectory.getPositionAtTime(time).pos;
        auto camera = cameraMap[ballsSinceShot[i].cameraID].position();

        auto scale = camera.z() / (camera.z() - realPos.z());
        auto xPos = (realPos.x() - camera.x()) * scale + camera.x();
        auto yPos = (realPos.y() - camera.y()) * scale + camera.y();
        auto projectedPos = Eigen::Vector2d(xPos, yPos);
        double distance = (projectedPos - ballsSinceShot[i].position).norm();
        totalDistance += distance;
    }

    return totalDistance / ballsSinceShot.size();
}

double ChipEstimator::getAverageDistance() {
    if (ballsSinceShot.size() < 9) {
        // TODO: CHECK HOW MANY WE NEED
        // std::cout << "Not enough balls to estimate chip" << std::endl;
        return 0;
    }
    return getAverageDistance(bestShotPos, bestShotVel, Eigen::Vector2d(0, 0));
}