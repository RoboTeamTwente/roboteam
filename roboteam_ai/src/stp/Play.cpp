#include "stp/Play.hpp"

#include "control/ControlUtils.h"
#include "gui/Out.h"

namespace rtt::ai::stp {

void Play::initialize() noexcept {
    stpInfos.clear();
    for (auto &role : roles) {
        if (role != nullptr) role->reset();
    }
    for (auto &role : roles) {
        if (role == nullptr) continue;
        stpInfos[role->getName()].setShouldAvoidBall(FieldComputations::getBallAvoidance());
        stpInfos[role->getName()].setMaxRobotVelocity(control::ControlUtils::getMaxVelocity(false));
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
        stpInfos[roles[currentMaxRobots]->getName()].setPositionToMoveTo(Vector2(0.0, -field.playArea.width() / 2));
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
        if (stpInfos.find(role->getName())->second.getRobot() &&
            stpInfos.find(role->getName())->second.getRobot()->get()->getId() == GameStateManager::getCurrentGameState().cardId &&
            (stpInfos.find(role->getName())->second.getRobot()->get()->getPos() - Vector2(0.0, -field.playArea.height() / 2)).length() <=
                control_constants::GO_TO_POS_ERROR_MARGIN * 4) {
            stpInfos[role->getName()].setShouldAvoidTheirRobots(false);
            stpInfos[role->getName()].setShouldAvoidOurRobots(false);
        }
    }
    DrawMargins();
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
            if (stpInfo->second.getRobot()) {
                stpInfo->second.setMaxRobotVelocity(control::ControlUtils::getMaxVelocity(stpInfo->second.getRobot().value()->hasBall()));
                stpInfo->second.setShouldAvoidBall(FieldComputations::getBallAvoidance());
            }

            // The keeper does not need to avoid our defense area
            if (stpInfo->second.getRoleName() == "keeper") stpInfo->second.setShouldAvoidOurDefenseArea(false);

            // Assign the new BallView and field
            stpInfo->second.setBall(newBallView);
            stpInfo->second.setField(field);
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
    int cardId = GameStateManager::getCurrentGameState().cardId;

    if (currentMaxRobots < sizeUs) {
        RTT_INFO("Driving robot ", roles[currentMaxRobots]->getName(), " to edge of field for substitution")
        flagMap[roles[currentMaxRobots]->getName()].priority = DealerFlagPriority::CARD;
        flagMap[roles[currentMaxRobots]->getName()].forcedID = cardId;
        stpInfos[roles[currentMaxRobots]->getName()].setShouldAvoidBall(true);
        stpInfos[roles[currentMaxRobots]->getName()].setPositionToMoveTo(Vector2(0.0, -field.playArea.height() / 2));
    }
    // Only keep the first n roles, where n is the amount of robots we have
    // This order is based on the order of the roles array
    for (size_t i = sizeUs; i < roles.size(); i++) {
        flagMap.erase(roles[i]->getName());
    }
    auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap, stpInfos);

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
                GameStateManager::getCurrentGameState().cardId = robot->getId();
                RTT_INFO("Updating cardId to: ", GameStateManager::getCurrentGameState().cardId)
                cardIdAssigned = true;
            }
        }
    }
    if (!cardIdAssigned && cardId != -1) {
        RTT_INFO("No robot assigned to cardId, resetting to -1")
        GameStateManager::getCurrentGameState().cardId = -1;
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

void Play::DrawMargins() noexcept {
    RuleSetName ruleSetTitle = GameStateManager::getCurrentGameState().getRuleSet().getTitle();
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();
    std::array<rtt::Vector2, 4> rightDefenseAreaMargin = {field.rightDefenseArea.topRight() + Vector2(0.0, 0.2), field.rightDefenseArea.topLeft() + Vector2(-0.2, 0.2),
                                                          field.rightDefenseArea.bottomLeft() + Vector2(-0.2, -0.2), field.rightDefenseArea.bottomRight() + Vector2(0.0, -0.2)};
    std::array<rtt::Vector2, 4> leftDefenseAreaMargin = {field.leftDefenseArea.topLeft() + Vector2(0.0, 0.2), field.leftDefenseArea.topRight() + Vector2(0.2, 0.2),
                                                         field.leftDefenseArea.bottomRight() + Vector2(0.2, -0.2), field.leftDefenseArea.bottomLeft() + Vector2(0.0, -0.2)};
    std::array<rtt::Vector2, 1> cardId = {rtt::Vector2(0.0, -field.playArea.height() / 2)};
    std::array<rtt::Vector2, 1> placementLocation = {rtt::ai::GameStateManager::getRefereeDesignatedPosition()};
    std::array<rtt::Vector2, 2> pathToPlacementLocation = {world->getWorld()->getBall()->get()->position, world->getWorld()->getBall()->get()->position};

    if (currentGameState == RefCommand::PENALTY_US || currentGameState == RefCommand::PENALTY_THEM || currentGameState == RefCommand::PREPARE_PENALTY_THEM ||
        currentGameState == RefCommand::PREPARE_PENALTY_US) {
        std::array<rtt::Vector2, 1> penaltyUs = {rtt::Vector2(2.0, 0.0)};
        if (currentGameState == RefCommand::PENALTY_US || currentGameState == RefCommand::PREPARE_PENALTY_US) {
            penaltyUs = {rtt::Vector2(-2.0, 0.0)};
        }
        rtt::ai::gui::Out::draw(
            {
                .label = "Penalty Place",
                .color = proto::Drawing::BLACK,
                .method = proto::Drawing::CIRCLES,
                .category = proto::Drawing::MARGINS,
                .size = 12,
                .thickness = 4,
            },
            penaltyUs);
    }

    if (currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT ||
        currentGameState == RefCommand::PREPARE_FORCED_START)
        pathToPlacementLocation = {world->getWorld()->getBall()->get()->position, rtt::ai::GameStateManager::getRefereeDesignatedPosition()};

    // Drawing all figures regarding states robots have to avoid certain area's (stop, ball placement, free kick, kick off)
    if (ruleSetTitle == RuleSetName::STOP || currentGameState == RefCommand::DIRECT_FREE_THEM || currentGameState == RefCommand::DIRECT_FREE_THEM_STOP ||
        currentGameState == RefCommand::DIRECT_FREE_US || currentGameState == RefCommand::KICKOFF_US || currentGameState == RefCommand::KICKOFF_THEM ||
        currentGameState == RefCommand::PREPARE_FORCED_START || currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::BALL_PLACEMENT_US ||
        currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT) {
        if (currentGameState != RefCommand::PREPARE_FORCED_START) {
            rtt::ai::gui::Out::draw(
                {
                    .label = "Left defense area to avoid",
                    .color = GameSettings::isYellow() ? proto::Drawing::BLUE : proto::Drawing::YELLOW,
                    .method = proto::Drawing::LINES_CONNECTED,
                    .category = proto::Drawing::MARGINS,
                    .size = 8,
                    .thickness = 8,
                },
                leftDefenseAreaMargin);
            rtt::ai::gui::Out::draw(
                {
                    .label = "Right defense area to avoid",
                    .color = GameSettings::isYellow() ? proto::Drawing::YELLOW : proto::Drawing::BLUE,
                    .method = proto::Drawing::LINES_CONNECTED,
                    .category = proto::Drawing::MARGINS,
                    .size = 8,
                    .thickness = 8,
                },
                rightDefenseAreaMargin);
        }
        proto::Drawing::Color color;
        if (currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::DIRECT_FREE_THEM || currentGameState == RefCommand::KICKOFF_THEM)
            color = GameSettings::isYellow() ? proto::Drawing::YELLOW : proto::Drawing::BLUE;
        else if (currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT || currentGameState == RefCommand::DIRECT_FREE_US ||
                 currentGameState == RefCommand::KICKOFF_US)
            color = GameSettings::isYellow() ? proto::Drawing::BLUE : proto::Drawing::YELLOW;
        else
            color = proto::Drawing::RED;
        for (auto method : {proto::Drawing::CIRCLES, proto::Drawing::LINES_CONNECTED}) {
            rtt::ai::gui::Out::draw(
                {
                    .label = method == proto::Drawing::CIRCLES ? "Ball area to avoid" : "Path to placement location",
                    .color = method == proto::Drawing::CIRCLES ? color : proto::Drawing::BLACK,
                    .method = method,
                    .category = proto::Drawing::MARGINS,
                    .size = method == proto::Drawing::CIRCLES ? 52 : 8,
                    .thickness = method == proto::Drawing::CIRCLES ? 4 : 8,
                },
                pathToPlacementLocation);
        }
    }

    // Drawing all figures regarding ball placement location and the path towards it
    if (currentGameState == RefCommand::BALL_PLACEMENT_THEM || currentGameState == RefCommand::BALL_PLACEMENT_US || currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT ||
        currentGameState == RefCommand::PREPARE_FORCED_START) {
        rtt::ai::gui::Out::draw(
            {
                .label = "Placement location",
                .color = proto::Drawing::BLACK,
                .method = proto::Drawing::CIRCLES,
                .category = proto::Drawing::MARGINS,
                .size = 17,
                .thickness = 4,
            },
            placementLocation);
    }

    if (GameStateManager::getCurrentGameState().cardId != -1) {
        rtt::ai::gui::Out::draw(
            {
                .label = "CardID",
                .color = proto::Drawing::BLACK,
                .method = proto::Drawing::CIRCLES,
                .category = proto::Drawing::MARGINS,
                .size = 15,
                .thickness = 7,
            },
            cardId);
    }
    std::array<std::string, 4> names = {"harasser", "passer", "receiver", "striker"};
    std::array<proto::Drawing::Color, 4> colors = {proto::Drawing::RED, proto::Drawing::WHITE, proto::Drawing::MAGENTA, proto::Drawing::WHITE};
    for (std::size_t i = 0; i < names.size(); i++) {
        if (stpInfos[names[i]].getRobot()) {
            std::array<rtt::Vector2, 1> position = {stpInfos[names[i]].getRobot()->get()->getPos()};
            rtt::ai::gui::Out::draw(
                {
                    .label = names[i],
                    .color = colors[i],
                    .method = proto::Drawing::CIRCLES,
                    .category = proto::Drawing::DEBUG,
                    .size = 15,
                    .thickness = 7,
                },
                position);
        }
    }
}

}  // namespace rtt::ai::stp
