//
// Created by mrlukasbos on 19-2-19.
//

#include <control/ControlUtils.h>
#include <world/BallPossession.h>
#include "analysis/GameAnalyzer.h"
#include "world/World.h"
#include "world/Field.h"
#include "world/Robot.h"
#include "analysis/RobotDanger.h"

namespace rtt {
namespace ai {
namespace analysis {

GameAnalyzer::GameAnalyzer()
        :running(false), stopping(false) { }

GameAnalyzer &GameAnalyzer::getInstance() {
    static GameAnalyzer instance;
    return instance;
}

/// Generate a report with the game analysis
std::shared_ptr<AnalysisReport> GameAnalyzer::generateReportNow() {

    if (world::world->weHaveRobots()) {
        std::shared_ptr<AnalysisReport> report = std::make_shared<AnalysisReport>();

        report->ballPossession = convertPossession(ballPossessionPtr->getPossession());
        report->ourDistanceToGoalAvg = getTeamDistanceToGoalAvg(true);
        report->theirDistanceToGoalAvg = getTeamDistanceToGoalAvg(false);
        report->theirRobotSortedOnDanger = getRobotsSortedOnDanger(false);
        report->ourRobotsSortedOnDanger = getRobotsSortedOnDanger(true);

        std::lock_guard<std::mutex> lock(mutex);
        mostRecentReport = report;
        return report;
    }
    std::cout << "NOT A WORLD YET" << std::endl;
    start(30);
    return {};
}

BallPossession GameAnalyzer::convertPossession(rtt::ai::BallPossession::Possession possession) {
    switch (possession) {
    default:
    case (rtt::ai::BallPossession::LOOSEBALL): {
        auto ballPosX = rtt::ai::world::world->getBall()->getPos().x;
        if (ballPosX > 0) {
            return BallPossession::OFFENSIVE_NEUTRAL;
        }
        else{
            return BallPossession::DEFENSIVE_NEUTRAL;
        }
    }
    case (rtt::ai::BallPossession::CONTENDEDBALL): return BallPossession::NEUTRAL;
    case (rtt::ai::BallPossession::THEIRBALL): return BallPossession::THEY_HAVE_BALL;
    case (rtt::ai::BallPossession::OURBALL): return BallPossession::WE_HAVE_BALL;
    }
}

/// Get the average of the distances of robots to their opponents goal
double GameAnalyzer::getTeamDistanceToGoalAvg(bool ourTeam, WorldData simulatedWorld) {
    auto robots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    double total = 0.0;
    for (auto robot : robots) {
        total += world::field->getDistanceToGoal(ourTeam, robot->pos);
    }
    return (total/robots.size());
}

/// return the attackers of a given team sorted on their vision on their opponents goal
std::vector<std::pair<GameAnalyzer::RobotPtr, double>> GameAnalyzer::getAttackersSortedOnGoalVision(bool ourTeam,
        WorldData simulatedWorld) {
    auto robots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    std::vector<std::pair<RobotPtr, double>> robotsWithVisibilities;

    for (auto robot : robots) {
        robotsWithVisibilities.emplace_back(robot,
                world::field->getPercentageOfGoalVisibleFromPoint(! ourTeam, robot->pos, world::world->getWorld()));
    }

    // sort on goal visibility
    std::sort(robotsWithVisibilities.begin(), robotsWithVisibilities.end(),
            [](std::pair<RobotPtr, double> a, std::pair<RobotPtr, double> b) {
              return a.second < b.second;
            });

    return robotsWithVisibilities;
}

/// return the average goal vision of a given team towards their opponents goal
double GameAnalyzer::getTeamGoalVisionAvg(bool ourTeam, WorldData simulatedWorld) {
    auto robots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    double total = 0.0;
    for (auto robot : robots) {
        total += world::field->getPercentageOfGoalVisibleFromPoint(! ourTeam, robot->pos, world::world->getWorld());
    }
    return (total/robots.size());
}

/// returns a danger score
RobotDanger GameAnalyzer::evaluateRobotDangerScore(RobotPtr robot, bool ourTeam) {
    Vector2 goalCenter = ourTeam ? world::field->get_field()[OUR_GOAL_CENTER] :
            world::field->get_field()[THEIR_GOAL_CENTER];

    RobotDanger danger;
    danger.ourTeam = ourTeam;
    danger.id = robot->id;
    danger.distanceToGoal = world::field->getDistanceToGoal(ourTeam, robot->pos);
    danger.shortestDistToEnemy = shortestDistToEnemyRobot(robot, ourTeam);
    danger.goalVisionPercentage = world::field->getPercentageOfGoalVisibleFromPoint(! ourTeam, robot->pos, world::world->getWorld());
    danger.robotsToPassTo = getRobotsToPassTo(robot, ourTeam);
    danger.closingInToGoal = isClosingInToGoal(robot, ourTeam);
    danger.aimedAtGoal = control::ControlUtils::robotIsAimedAtPoint(robot->id, ourTeam, goalCenter);

    return danger;
}

std::shared_ptr<AnalysisReport> GameAnalyzer::getMostRecentReport() {
    std::lock_guard<std::mutex> lock(mutex);
    return mostRecentReport;
}

/// Check with distanceToLineWithEnds if there are obstructions
/// Returns all robots that can be passed to, along with the distance
/// we return robot ids instead of robot object because the objects are incorrect (because of simulated world)
std::vector<std::pair<int, double>> GameAnalyzer::getRobotsToPassTo(RobotPtr robot, bool ourTeam, WorldData simulatedWorld) {
    auto ourRobots = ourTeam ? simulatedWorld.us : simulatedWorld.them;
    auto enemyRobots = ourTeam ? simulatedWorld.them : simulatedWorld.us;

    std::vector<std::pair<int, double>> robotsToPassTo;
    for (auto ourRobot : ourRobots) {
        bool canPassToThisRobot = true;
        for (auto theirRobot : enemyRobots) {
            auto distToLine = control::ControlUtils::distanceToLineWithEnds(theirRobot->pos, Vector2(robot->pos),
                    Vector2(ourRobot->pos));
            if (distToLine < (Constants::ROBOT_RADIUS_MAX() + Constants::BALL_RADIUS())) {
                canPassToThisRobot = false;
                break;
            }
        }
        if (canPassToThisRobot) {
            double distToRobot = (Vector2(ourRobot->pos) - Vector2(robot->pos)).length();
            robotsToPassTo.emplace_back(std::make_pair(ourRobot->id, distToRobot));
        }
    }
    return robotsToPassTo;
}

/// get the shortest distance to an enemy robot
/// this is useful to check if a robot stands free
double GameAnalyzer::shortestDistToEnemyRobot(RobotPtr robot, bool ourTeam, WorldData simulatedWorld) {
    auto enemyRobots = ourTeam ? simulatedWorld.them : simulatedWorld.us;
    Vector2 robotPos = robot->pos;
    double shortestDist = INT_MAX;
    for (auto opponent : enemyRobots) {
        shortestDist = std::min(robotPos.dist(opponent->pos), shortestDist);
    }
    return shortestDist;
}

/// check if a robot is closing in to our goal.
bool GameAnalyzer::isClosingInToGoal(RobotPtr robot, bool ourTeam) {
    double distanceToGoal = world::field->getDistanceToGoal(ourTeam, robot->pos);

    WorldData futureWorld = world::world->getFutureWorld(0.2);
    auto enemyRobots = ourTeam ? futureWorld.them : futureWorld.us;

    // find the robot in the future and look if it is closer than before
    for (auto futureRobot : enemyRobots) {
        if (futureRobot->id == robot->id && world::field->getDistanceToGoal(ourTeam, futureRobot->pos) < distanceToGoal) {
            return true;
        }
    }
    return false;
}

void GameAnalyzer::start(int iterationsPerSecond) {
    if (! running && world::world->weHaveRobots()) {
        std::cout << "GameAnalyzer: " << "Starting at " << iterationsPerSecond << " iterations per second" << std::endl;
        auto delay = (unsigned) (1000.0/iterationsPerSecond);
        thread = std::thread(&GameAnalyzer::loop, this, delay);
        running = true;
    }
}

// Stops the background worker thread.
void GameAnalyzer::stop() {
    stopping = true;
    if (running || stopping) {
        std::cout << "GameAnalyzer: " << "Stopping GameAnalyzer" << std::endl;
        thread.join();
        running = false;
        stopping = false;
    }
    else {
        std::cout << "GameAnalyzer: " << "Could not stop since it was not running in the first place." << std::endl;
    }
}

void GameAnalyzer::loop(unsigned delayMillis) {
    std::chrono::milliseconds delay(delayMillis);
    while (! stopping) {
        generateReportNow();
        std::this_thread::sleep_for(delay);
    }
}

std::vector<std::pair<GameAnalyzer::RobotPtr, RobotDanger>> GameAnalyzer::getRobotsSortedOnDanger(bool ourTeam) {
    auto robots = ourTeam ? world::world->getUs() : world::world->getThem();
    std::vector<std::pair<RobotPtr, RobotDanger>> robotDangers;

    for (auto robot : robots) {
        robotDangers.emplace_back(robot, evaluateRobotDangerScore(robot, ourTeam));
    }

    std::sort(robotDangers.begin(), robotDangers.end(),
            [](std::pair<RobotPtr, RobotDanger> a,
                    std::pair<RobotPtr, RobotDanger> b) {
              return a.second.getTotalDanger() > b.second.getTotalDanger();
            });

    return robotDangers;
}

} // analysis
} // ai
} // rtt