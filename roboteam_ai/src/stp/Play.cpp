//
// Created by john on 3/9/20.
//

#include "stp/Play.hpp"

#include "control/ControlUtils.h"

namespace rtt::ai::stp {

void Play::initialize() noexcept {
    stpInfos.clear();
    for (auto &role : roles) {
        if (role != nullptr) role->reset();
    }
    for (auto &role : roles) {
        if (role == nullptr) continue;
        stpInfos[role->getName()].setShouldAvoidBall(FieldComputations::getBallAvoidance());
    }
    calculateInfoForRoles();
    distributeRoles();
    previousRobotNum = world->getWorld()->getRobotsNonOwning().size();
    previousKeeperId = GameStateManager::getCurrentGameState().keeperId;
    previousMaxRobots = GameStateManager::getCurrentGameState().maxAllowedRobots;
}

void Play::setWorld(world::World *world) noexcept { this->world = world; }

void Play::updateField(Field field) noexcept { this->field = field; }

void Play::update() noexcept {
    // clear roleStatuses so it only contains the current tick's statuses
    roleStatuses.clear();
    //    RTT_INFO("Play executing: ", getName())

    // Check if the amount of robots changed or keeper id changed
    // If so, we will re deal the roles
    auto currentRobotNum{world->getWorld()->getRobotsNonOwning().size()};
    auto currentKeeperId = GameStateManager::getCurrentGameState().keeperId;
    auto currentMaxRobots = GameStateManager::getCurrentGameState().maxAllowedRobots;
    int sizeUs = world->getWorld()->getUs().size();

    // We want to redeal the roles if the amount of robots changed or the keeper id changed. Also if we get a yellow card (current max goes down) and we have more robots than
    // allowed, or if a yellow card expires (current max goes up) and we have more robots than previously allowed on the field (some robot was still driving to the edge of the
    // field)
    if (currentRobotNum != previousRobotNum || currentKeeperId != previousKeeperId || (currentMaxRobots < previousMaxRobots && currentMaxRobots < sizeUs) ||
        (currentMaxRobots > previousMaxRobots && sizeUs > previousMaxRobots)) {
        // RTT_INFO("Reassigning bots")
        reassignRobots();
        previousRobotNum = currentRobotNum;
        previousKeeperId = currentKeeperId;
    }
    previousMaxRobots = currentMaxRobots;

    // Refresh the RobotViews, BallViews and fields
    refreshData();

    // derived class method call
    calculateInfoForRoles();

    sizeUs = std::min(sizeUs, static_cast<int>(roles.size()));

    // If we have more robots than allowed, one drives to the edge of the field
    if (currentMaxRobots < sizeUs) {
        stpInfos[roles[currentMaxRobots]->getName()].setShouldAvoidBall(true);
        stpInfos[roles[currentMaxRobots]->getName()].setPositionToMoveTo(Vector2(0, -field.playArea.width() / 2));
    }

    // Loop through roles and update them if they are assigned to a robot
    for (auto &role : roles) {
        if (role == nullptr) continue;
        auto stpInfo = stpInfos.find(role->getName());
        if (stpInfo != stpInfos.end() && stpInfo->second.getRobot()) {
            // Update and store the returned status
            auto roleStatus = role->update(stpInfos[role->getName()]);
            roleStatuses[role.get()] = roleStatus;
        }
    }
}

void Play::reassignRobots() noexcept {
    stpInfos.clear();
    calculateInfoForRoles();
    distributeRoles();
}

void Play::refreshData() noexcept {
    // Get a new BallView and field from world
    auto newBallView = world->getWorld()->getBall();

    // Loop through all roles, if an stpInfo exists and has an assigned robot, refresh the data
    for (auto &role : roles) {
        if (role == nullptr) continue;
        auto stpInfo = stpInfos.find(role->getName());
        if (stpInfo != stpInfos.end() && stpInfo->second.getRobot().has_value()) {
            // Get a new RobotView from world using the old robot id
            stpInfo->second.setRobot(world->getWorld()->getRobotForId(stpInfo->second.getRobot()->get()->getId()));

            // Set max velocity depending on the gamestate rules and whether we have the ball
            if (stpInfo->second.getRobot()) stpInfo->second.setMaxRobotVelocity(control::ControlUtils::getMaxVelocity(stpInfo->second.getRobot().value()->hasBall()));

            // The keeper does not need to avoid our defense area
            if (stpInfo->second.getRoleName() == "keeper") stpInfo->second.setShouldAvoidDefenseArea(false);

            // Assign the new BallView and field
            stpInfo->second.setBall(newBallView);
            stpInfo->second.setField(field);

            if (stpInfo->second.getEnemyRobot().has_value()) {
                stpInfo->second.setEnemyRobot(world->getWorld()->getRobotForId(stpInfo->second.getEnemyRobot()->get()->getId(), false));
            }
        }
    }
}

void Play::distributeRoles() noexcept {
    Dealer dealer{world->getWorld().value(), &field};

    // Set role names for each stpInfo
    for (auto &role : roles) {
        if (role == nullptr) continue;
        auto roleName{role->getName()};
        stpInfos[roleName].setRoleName(roleName);
    }

    auto flagMap = decideRoleFlags();

    int currentMaxRobots = GameStateManager::getCurrentGameState().maxAllowedRobots;
    int sizeUs = world->getWorld()->getUs().size();
    sizeUs = std::min(sizeUs, static_cast<int>(roles.size()));
    static int cardId = -1;

    if (currentMaxRobots < sizeUs) {
        RTT_INFO("Driving robot ", roles[currentMaxRobots]->getName(), " to edge of field for substitution")
        flagMap[roles[currentMaxRobots]->getName()].priority = DealerFlagPriority::CARD;
        flagMap[roles[currentMaxRobots]->getName()].forcedID = cardId;
        stpInfos[roles[currentMaxRobots]->getName()].setShouldAvoidBall(true);
        stpInfos[roles[currentMaxRobots]->getName()].setPositionToMoveTo(Vector2(0, -field.playArea.width() / 2));
    }
    // Only keep the first n roles, where n is the amount of robots we have
    // This order is based on the order of the roles array
    for (int i = sizeUs; i < roles.size(); i++) {
        flagMap.erase(roles[i]->getName());
    }
    auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap, stpInfos);

    // TODO-Max if role exists in oldStpInfos then copy those.
    // Clear the stpInfos for the new role assignment
    bool cardIdAssigned = false;
    for (auto &role : roles) {
        if (role == nullptr) continue;
        role->reset();
        auto roleName{role->getName()};
        if (distribution.find(roleName) != distribution.end()) {
            auto robot = distribution.find(role->getName())->second;
            stpInfos[roleName].setRobot(robot);
            if (flagMap[roleName].priority == DealerFlagPriority::CARD) {
                cardId = robot->getId();
                RTT_INFO("Updating cardId to: ", cardId)
                cardIdAssigned = true;
            }
        }
    }
    if (!cardIdAssigned && cardId != -1) {
        RTT_INFO("No robot assigned to cardId, resetting to -1")
        cardId = -1;
    }
    std::for_each(stpInfos.begin(), stpInfos.end(), [this](auto &each) { each.second.setCurrentWorld(world); });
}

std::unordered_map<Role *, Status> const &Play::getRoleStatuses() const { return roleStatuses; }

bool Play::isValidPlayToKeep() noexcept {
    return (!shouldEndPlay() && std::all_of(keepPlayEvaluation.begin(), keepPlayEvaluation.end(), [this](auto &x) { return PlayEvaluator::checkEvaluation(x, world); }));
}

bool Play::isValidPlayToStart() const noexcept {
    return std::all_of(startPlayEvaluation.begin(), startPlayEvaluation.end(), [this](auto &x) { return PlayEvaluator::checkEvaluation(x, world); });
}

uint8_t Play::getLastScore() const { return lastScore.value_or(0); }

bool Play::shouldEndPlay() noexcept { return false; }

}  // namespace rtt::ai::stp
