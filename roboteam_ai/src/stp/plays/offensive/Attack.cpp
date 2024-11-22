#include <iostream>
#include <thread>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <vector>
#include <array>
#include "stp/plays/offensive/Attack.h"
#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Striker.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "../../../roboteam_networking/proto/ActionCommand.pb.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::WeWillHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles.push_back(std::make_unique<role::Keeper>("keeper"));
    roles.push_back(std::make_unique<role::Striker>("striker"));
    roles.push_back(std::make_unique<role::Defender>("defender_0"));
    roles.push_back(std::make_unique<role::Formation>("attacker_0"));
    roles.push_back(std::make_unique<role::Defender>("defender_1"));
    roles.push_back(std::make_unique<role::Defender>("defender_2"));
}

Attack::~Attack() = default;

void Attack::receiveActionCommand() {
    const char* fifo_path = "/tmp/action_pipe";
    mkfifo(fifo_path, 0666);  // Create the FIFO

    while (true) {
        int fifo_fd = open(fifo_path, O_RDONLY);
        if (fifo_fd == -1) {
            perror("Error opening FIFO");
            return;
        }

        char buffer[1024];
        ssize_t bytes_read = read(fifo_fd, buffer, sizeof(buffer));
        close(fifo_fd);

        if (bytes_read > 0) {
            ActionCommand action_command;
            if (action_command.ParseFromArray(buffer, bytes_read)) {
                int num_attacker = action_command.numattacker();
                int num_defender = action_command.numdefender();
                int num_waller = action_command.numwaller();

                roles.clear();  // Clear existing roles

                for (int i = 0; i < num_attacker; ++i) {
                    roles.push_back(std::make_unique<role::Striker>("striker_" + std::to_string(i)));
                }
                for (int i = 0; i < num_defender; ++i) {
                    roles.push_back(std::make_unique<role::Defender>("defender_" + std::to_string(i)));
                }
                for (int i = 0; i < num_waller; ++i) {
                    roles.push_back(std::make_unique<role::Defender>("waller_" + std::to_string(i)));
                }

                std::cout << "Received: numDefender=" << num_defender
                          << ", numAttacker=" << num_attacker
                          << ", numWaller=" << num_waller << std::endl;
            } else {
                std::cerr << "Failed to parse protobuf message" << std::endl;
            }
        }
    }
}

uint8_t Attack::score(const rtt::Field& field) noexcept {
    return PositionScoring::scorePosition(
               world->getWorld()->getBall().value()->position, gen::GoalShot, field, world)
               .score *
           (rand() % 2 + 1);
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"striker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"waller_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    std::array<std::unique_ptr<Role>, constants::MAX_ROBOT_COUNT> roles_array{};
    std::move(roles.begin(), roles.end(), roles_array.begin());

    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles_array, field, world, true);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles_array, field, world);
}

bool Attack::shouldEndPlay() noexcept {
    if (std::any_of(roles.begin(), roles.end(), [](const auto& role) {
            return role != nullptr && role->getName() == "striker" && role->finished();
        })) {
        return true;
    }

    auto strikerId = stpInfos.at("striker");
    if (strikerId.getRobot() && strikerId.getRobot().value()) {
        auto strikerRobotId = strikerId.getRobot().value()->getId();
        if (InterceptionComputations::calculateInterceptionInfoExcludingKeeperAndCarded(world).interceptId != strikerRobotId) {
            return true;
        }
    }

    return false;
}

const char* Attack::getName() const { return "Attack"; }

}  // namespace rtt::ai::stp::play
