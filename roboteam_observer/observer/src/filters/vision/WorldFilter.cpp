#include "filters/vision/WorldFilter.h"

#include "filters/vision/ball/BallAssigner.h"
WorldFilter::WorldFilter() {}

proto::World WorldFilter::getWorldPrediction(const Time &time) const {
    proto::World world;
    addRobotPredictionsToMessage(world, time);
    addBallPredictionsToMessage(world, time);
    world.set_time(time.asNanoSeconds());

    return world;
}

void WorldFilter::process(const std::vector<proto::SSL_DetectionFrame> &frames, const std::vector<rtt::RobotsFeedback> &feedback, const std::vector<int> &camera_ids) {
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
        bool allSmall = std::all_of(oneIDFilters.second.begin(), oneIDFilters.second.end(), [](const RobotFilter &filter) { return filter.getNumObservations() <= 3; });
        if (allSmall) {
            continue;
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
    int blueCount = 0;
    for (const auto &blueBot : blueRobots) {
        if (blueCount >= 11) {
            break;  // Only 11 robots per team, if vision is bad, we might have more robots, making ai crash
        }
        blueCount++;
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
    int yellowCount = 0;
    for (const auto &yellowBot : yellowRobots) {
        if (yellowCount >= 11) {
            break;  // Only 11 robots per team, if vision is bad, we might have more robots, making ai crash
        }
        yellowCount++;
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
    for (std::size_t i = 0; i < balls.size(); ++i) {
        predictions[i] = balls[i].predictCam(frame.cameraID, frame.timeCaptured).prediction;
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
void WorldFilter::addBallPredictionsToMessage(proto::World &world, Time time) const {
    const BallFilter *bestFilter = nullptr;
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
    world.mutable_ball()->CopyFrom(bestBall.asWorldBall());
}
