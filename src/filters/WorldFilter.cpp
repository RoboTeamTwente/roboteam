//
// Created by kjhertenberg on 13-5-19.
//

#include <filters/WorldFilter.h>
#include <memory>
#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"

namespace world {

WorldFilter::WorldFilter() {
    blueBots.clear();
    yellowBots.clear();
    balls.clear();
}

// if we get a new frame we update our observations
void WorldFilter::addFrame(const proto::SSL_DetectionFrame &msg) {
    const double filterGrabDistance = 0.5;
    double timeCapture = msg.t_capture();
    uint cameraID = msg.camera_id();
    handleRobots(yellowBots, msg.robots_yellow(), filterGrabDistance, timeCapture, cameraID);
    handleRobots(blueBots, msg.robots_blue(), filterGrabDistance, timeCapture, cameraID);
    handleBall(msg.balls(), filterGrabDistance, timeCapture, cameraID);
}
void WorldFilter::handleBall(const google::protobuf::RepeatedPtrField<proto::SSL_DetectionBall> &observations, const double filterGrabDistance, double timeCapture, uint cameraID) {
    for (const proto::SSL_DetectionBall &detBall : observations) {
        bool addedBall = false;
        for (const auto &filter : balls) {
            if (filter->distanceTo(detBall.x(), detBall.y()) < filterGrabDistance) {
                filter->addObservation(detBall, timeCapture, cameraID);
                addedBall = true;
            }
        }
        if (!addedBall) {
            // We create a new filter if there is no existing filter which is reasonably close to the detection
            balls.push_back(std::make_unique<BallFilter>(detBall, timeCapture, cameraID));
        }
    }
}
void WorldFilter::handleRobots(robotMap &robots, const google::protobuf::RepeatedPtrField<proto::SSL_DetectionRobot> &observations, double filterGrabDistance, double timeCapture,
                               uint cameraID) {
    for (const proto::SSL_DetectionRobot &robot : observations) {
        bool addedBot = false;
        for (const auto &filter : robots[robot.robot_id()]) {
            if (filter->distanceTo(robot.x(), robot.y()) < filterGrabDistance) {
                filter->addObservation(robot, timeCapture, cameraID);
                addedBot = true;
            }
        }
        if (!addedBot) {
            // We create a new filter if there is no existing filter which is reasonably close to the detection
            robots[robot.robot_id()].push_back(std::make_unique<RobotFilter>(robot, timeCapture, cameraID));
        }
    }
}

// Creates a world message with the currently observed objects in it
proto::World WorldFilter::getWorld(double time) {
    // First we update to the time we want packets at. Expensive, but ensures we have the latest information
    update(time, true);
    proto::World world;
    world.set_time(time);
    for (const auto &yellowBotsOneId : yellowBots) {
        if (!yellowBotsOneId.second.empty()) {
            world.mutable_yellow()->Add(bestFilter(yellowBotsOneId.second)->asWorldRobot());
        }
    }
    for (const auto &blueBotsOneId : blueBots) {
        if (!blueBotsOneId.second.empty()) {
            world.mutable_blue()->Add(bestFilter(blueBotsOneId.second)->asWorldRobot());
        }
    }
    if (!balls.empty()) {
        proto::WorldBall worldBall = bestFilter(balls)->asWorldBall();
        world.mutable_ball()->CopyFrom(worldBall);
    }
    return world;
}
void WorldFilter::update(double time, bool doLastPredict) {
    const double removeFilterTime = 0.4;  // Remove filters if no new observations have been added to it for this amount of time
    updateRobots(yellowBots, time, doLastPredict, removeFilterTime);
    updateRobots(blueBots, time, doLastPredict, removeFilterTime);
    updateBalls(time, doLastPredict, removeFilterTime);
}
void WorldFilter::updateBalls(double time, bool doLastPredict, const double removeFilterTime) {
    auto ball = balls.begin();
    while (ball != balls.end()) {
        ball->get()->update(time, doLastPredict);
        if (time - ball->get()->getLastUpdateTime() > removeFilterTime) {
            balls.erase(ball);
        } else {
            ++ball;
        }
    }
}
void WorldFilter::updateRobots(robotMap &robots, double time, bool doLastPredict, double removeFilterTime) {
    for (auto &botsOneId : robots) {
        auto filter = botsOneId.second.begin();
        while (filter != botsOneId.second.end()) {
            filter->get()->update(time, doLastPredict);
            if (time - filter->get()->getLastUpdateTime() > removeFilterTime) {
                botsOneId.second.erase(filter);
            } else {
                ++filter;
            }
        }
    }
}
const std::unique_ptr<RobotFilter> &WorldFilter::bestFilter(const std::vector<std::unique_ptr<RobotFilter>> &filters) {
    int bestIndex = 0;
    int bestFrames = -1;
    for (int i = 0; i < filters.size(); ++i) {
        if (filters[i]->frames() > bestFrames) {
            bestFrames = filters[i]->frames();
            bestIndex = i;
        }
    }
    return filters[bestIndex];
}
const std::unique_ptr<BallFilter> &WorldFilter::bestFilter(const std::vector<std::unique_ptr<BallFilter>> &filters) {
    int bestIndex = 0;
    int bestFrames = -1;
    for (int i = 0; i < filters.size(); ++i) {
        if (filters[i]->frames() > bestFrames) {
            bestFrames = filters[i]->frames();
            bestIndex = i;
        }
    }
    return filters[bestIndex];
}
}  // namespace world