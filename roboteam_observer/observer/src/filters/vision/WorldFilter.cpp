#include "filters/vision/WorldFilter.h"

#include "filters/vision/ball/BallAssigner.h"
WorldFilter::WorldFilter() {}

proto::World WorldFilter::getWorldPrediction(const Time &time) {
    proto::World world;
    addRobotPredictionsToMessage(world, time);
    addBallPredictionsToMessage(world, time);
    world.set_time(time.asNanoSeconds());

    return world;
}

void WorldFilter::process(const std::vector<proto::SSL_DetectionFrame> &frames, const std::vector<rtt::RobotsFeedback> &feedback, const std::vector<int> &camera_ids,
                          GeometryFilter &geomFilter) {
    // populate cameraMap
    for (const auto &camera : geomFilter.getCameras()) {
        if (!cameraMap.hasCamera(camera.first)) {
            cameraMap.addCamera(Camera(camera.second));
        }
    }
    // Feedback is processed first, as it is not really dependent on vision packets,
    // but the vision processing may be helped by the feedback information
    feedbackFilter.process(feedback);

    std::vector<DetectionFrame> detectionFrames;
    for (const auto &protoFrame : frames) {
        if (!camera_ids.empty() && std::find(camera_ids.begin(), camera_ids.end(), protoFrame.camera_id()) == camera_ids.end()) {
            continue;
        }
        detectionFrames.emplace_back(DetectionFrame(protoFrame));
    }
    // Sort by time
    std::sort(detectionFrames.begin(), detectionFrames.end(), [](const DetectionFrame &lhs, const DetectionFrame &rhs) { return lhs.timeCaptured < rhs.timeCaptured; });
    for (auto &frame : detectionFrames) {
        auto cameraTime = lastCaptureTimes.find(frame.cameraID);
        frame.dt = cameraTime == lastCaptureTimes.end() ? 0.0 : (frame.timeCaptured - cameraTime->second).asSeconds();

        // TODO: make a realtime option
        // TODO: remove any frames with captures times which differ more than a second from the current time
    }
    // Remove frames which are too late. For now we do this, because it's quite hard to go back in time and reconstruct the state of the entire visionFilter

    // This can also be caused by other teams running e.g. their simulators internally and accidentally broadcasting onto the network
    detectionFrames.erase(std::remove_if(detectionFrames.begin(), detectionFrames.end(), [](const DetectionFrame &frame) { return frame.dt < 0.0; }), detectionFrames.end());
    for (const auto &frame : detectionFrames) {
        lastCaptureTimes[frame.cameraID] = frame.timeCaptured;
    }
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
            if (it->processNotSeen(frame.cameraID, frame.timeCaptured)) {  // updates the relevant object filter. If the filter is redundant, we can remove it
                it = filters.erase(it);
            } else {
                it++;
            }
        }
    }
}

void WorldFilter::updateRobots(robotMap &robots, const std::vector<RobotObservation> &detectedRobots) {
    // add detected robots to existing filter(s) or create a new filter if no filter accepts the robot
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
    // robots are processed first, as the robots can affect the ball but vice versa not so much
    processRobots(frame, true);
    processRobots(frame, false);
    processBalls(frame);
}

void WorldFilter::updateRobotParameters(const TwoTeamRobotParameters &parameters) {
    blueParams = parameters.blueParameters;
    yellowParams = parameters.yellowParameters;
}

std::vector<FilteredRobot> WorldFilter::getHealthiestRobotsMerged(bool blueBots, Time time) const {
    std::vector<FilteredRobot> robots;
    const robotMap &map = blueBots ? blue : yellow;
    for (const auto &oneIDFilters : map) {
        if (oneIDFilters.second.empty()) {
            continue;
            ;
        }
        double maxHealth = -std::numeric_limits<double>::infinity();
        auto bestFilter = oneIDFilters.second.begin();
        for (auto robotFilter = oneIDFilters.second.begin(); robotFilter != oneIDFilters.second.end(); ++robotFilter) {
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
            continue;
            ;
        }
        double maxHealth = -std::numeric_limits<double>::infinity();
        auto bestFilter = oneIDFilters.second.begin();
        for (auto robotFilter = oneIDFilters.second.begin(); robotFilter != oneIDFilters.second.end(); ++robotFilter) {
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
void WorldFilter::addRobotPredictionsToMessage(proto::World &world, Time time) const {
    auto feedbackBots = feedbackFilter.getRecentFeedback();

    std::vector<FilteredRobot> blueRobots = getHealthiestRobotsMerged(true, time);
    for (const auto &blueBot : blueRobots) {
        auto worldBot = blueBot.asWorldRobot();
        auto feedback_it = std::find_if(feedbackBots.begin(), feedbackBots.end(), [&](const std::pair<TeamRobotID, proto::RobotProcessedFeedback> &bot) {
            return bot.first.team == TeamColor::BLUE && bot.first.robot_id == RobotID(worldBot.id());
        });
        if (feedback_it != feedbackBots.end()) {
            worldBot.mutable_feedbackinfo()->CopyFrom(feedback_it->second);
            feedbackBots.erase(feedback_it);
        }
        world.mutable_blue()->Add()->CopyFrom(worldBot);
    }
    std::vector<FilteredRobot> yellowRobots = getHealthiestRobotsMerged(false, time);
    for (const auto &yellowBot : yellowRobots) {
        auto worldBot = yellowBot.asWorldRobot();
        auto feedback_it = std::find_if(feedbackBots.begin(), feedbackBots.end(), [&](const std::pair<TeamRobotID, proto::RobotProcessedFeedback> &bot) {
            return bot.first.team == TeamColor::YELLOW && bot.first.robot_id == RobotID(worldBot.id());
        });
        if (feedback_it != feedbackBots.end()) {
            worldBot.mutable_feedbackinfo()->CopyFrom(feedback_it->second);
            feedbackBots.erase(feedback_it);
        }
        world.mutable_yellow()->Add()->CopyFrom(worldBot);
    }
    // Any remaining feedback of robots is put into the lonely category
    for (const auto &bot : feedbackBots) {
        auto *team_lonely_bots = bot.first.team == TeamColor::YELLOW ? world.mutable_yellow_unseen_robots() : world.mutable_blue_unseen_robots();
        proto::FeedbackOnlyRobot *robot = team_lonely_bots->Add();
        robot->set_id(bot.first.robot_id.robotID);
        robot->mutable_feedback()->CopyFrom(bot.second);
    }
}
void WorldFilter::processBalls(const DetectionFrame &frame) {
    std::vector<CameraGroundBallPrediction> predictions(balls.size());
    // get predictions from cameras
    std::vector<FilteredRobot> blueRobots = getHealthiestRobotsMerged(true, frame.timeCaptured);
    std::vector<FilteredRobot> yellowRobots = getHealthiestRobotsMerged(false, frame.timeCaptured);

    for (std::size_t i = 0; i < balls.size(); ++i) {
        predictions[i] = balls[i].predictCam(frame.cameraID, frame.timeCaptured, yellowRobots, blueRobots).prediction;
    }
    // assign observations to relevant filters
    BallAssignmentResult assignment = BallAssigner::assign_balls(predictions, frame.balls);

    // update filters with their paired observations
    // we iterate backwards: this helps keep the logic clean when erasing filters
    for (int i = static_cast<int>(balls.size()) - 1; i >= 0; --i) {
        bool removeFilter = balls[i].processDetections(assignment.op_pairs[i], frame.cameraID);
        if (removeFilter) {
            balls.erase(balls.begin() + i);
        }
    }

    // create new ball filters for balls which were not yet assigned
    for (const auto &newBall : assignment.unpairedObservations) {
        balls.emplace_back(BallFilter(newBall));
    }
}

void WorldFilter::kickDetector(FilteredBall bestBall, Time time) {
    // Check if there's no current observation, return early
    if (!bestBall.currentObservation.has_value() || bestBall.currentObservation.value().confidence < 0.1) {
        return;
    }

    // Get the healthiest robots for both teams
    std::vector<FilteredRobot> blueRobots = getHealthiestRobotsMerged(true, time);
    std::vector<FilteredRobot> yellowRobots = getHealthiestRobotsMerged(false, time);
    addRecentData(blueRobots, yellowRobots, bestBall);

    // If not enough frames in history, return early
    if (frameHistory.size() < 5) {
        return;
    }

    // Check if the last kick was too recent, return early
    if ((frameHistory.front().filteredBall->time - lastKickTime).asSeconds() < 0.1) {
        return;
    }

    // Gather all balls and robots within a certain proximity
    std::vector<FilteredBall> allBalls;
    std::map<TeamRobotID, std::vector<FilteredRobot>> allRobots;
    auto firstBallCameraPosition = frameHistory.front().filteredBall->positionCamera;
    for (const auto &data : frameHistory) {
        allBalls.push_back(data.filteredBall.value());
        for (const auto &robot : data.blue) {
            if ((robot.position.position - firstBallCameraPosition).norm() < 1.0) allRobots[robot.id].push_back(robot);
        }
        for (const auto &robot : data.yellow) {
            if ((robot.position.position - firstBallCameraPosition).norm() < 1.0) allRobots[robot.id].push_back(robot);
        }
    }

    // Remove robots if there's insufficient data
    for (auto it = allRobots.begin(); it != allRobots.end();) {
        if (it->second.size() < 5) {
            it = allRobots.erase(it);
        } else {
            ++it;
        }
    }

    for (const auto &[id, filteredRobots] : allRobots) {
        if (!checkDistance(filteredRobots, allBalls)) {
            continue;
        }
        if (!checkVelocity(allBalls)) {
            continue;
        }
        if (!checkOrientation(filteredRobots, allBalls)) {
            continue;
        }
        if (!checkIncreasingDistance(filteredRobots, allBalls)) {
            continue;
        }

        lastKickTime = frameHistory.front().filteredBall->time;
        mostRecentKick = KickEvent{id, filteredRobots[0].position, allBalls[0].positionCamera, lastKickTime, allBalls};
        // std::cout << "Kick detected by robot " << id.robot_id.robotID << " from team " << (id.team == TeamColor::BLUE ? "blue" : "yellow") << std::endl;
        break;
    }
}

bool WorldFilter::checkDistance(const std::vector<FilteredRobot> &robots, const std::vector<FilteredBall> &balls) {
    double initialDistance = (robots[0].position.position - balls[0].currentObservation.value().position).norm();
    double maxDistance = initialDistance;
    if (initialDistance > 0.17) return false;

    for (size_t i = 1; i < 5; ++i) {
        double distance = (robots[i].position.position - balls[i].currentObservation.value().position).norm();
        if (distance < initialDistance) return false;
        if (distance > maxDistance) maxDistance = distance;
    }

    return maxDistance >= 0.16;
}

bool WorldFilter::checkVelocity(const std::vector<FilteredBall> &balls) {
    int validVelocities = 0;
    for (size_t i = 1; i < 5; ++i) {
        auto previousBall = balls[i - 1].currentObservation.value();
        auto currentBall = balls[i].currentObservation.value();
        double velocity = (currentBall.position - previousBall.position).norm() / (currentBall.timeCaptured - previousBall.timeCaptured).asSeconds();
        if (velocity > 0.6) validVelocities++;
    }
    return validVelocities >= 2;
}

bool WorldFilter::checkOrientation(const std::vector<FilteredRobot> &robots, const std::vector<FilteredBall> &balls) {
    for (size_t i = 0; i < 5; ++i) {
        const auto &robot = robots[i];
        const auto &ballPosition = balls[i].currentObservation.value().position;
        Eigen::Vector2d robotToBall = ballPosition - robot.position.position;
        double angle = std::atan2(robotToBall.y(), robotToBall.x());
        double angleDiff = std::abs(robot.position.yaw - rtt::Angle(angle));
        if (angleDiff > 0.8) return false;
    }
    return true;
}

bool WorldFilter::checkIncreasingDistance(const std::vector<FilteredRobot> &robots, const std::vector<FilteredBall> &balls) {
    double previousDistance = (robots[0].position.position - balls[0].currentObservation.value().position).norm();
    for (size_t i = 1; i < 5; ++i) {
        double currentDistance = (robots[i].position.position - balls[i].currentObservation.value().position).norm();
        if (currentDistance < previousDistance) return false;
        previousDistance = currentDistance;
    }
    return true;
}

void WorldFilter::addBallPredictionsToMessage(proto::World &world, Time time) {
    BallFilter *bestFilter = nullptr;
    double bestHealth = -1.0;
    for (auto &filter : balls) {
        if (filter.getNumObservations() > 3) {
            double currentHealth = filter.getHealth();
            if (!bestFilter || currentHealth > bestHealth) {
                bestFilter = &filter;
                bestHealth = currentHealth;
            }
        }
    }
    if (!bestFilter) {
        return;
    }
    FilteredBall bestBall = bestFilter->mergeBalls(time);
    kickDetector(bestBall, time);

    world.mutable_ball()->CopyFrom(bestBall.asWorldBall());
}
