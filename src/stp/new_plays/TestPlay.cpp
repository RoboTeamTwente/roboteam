//
// Created by timovdk on 3/10/20.
//

#include <stp/new_plays/TestPlay.h>
#include <stp/new_roles/TestRole.h>

namespace rtt::ai::stp {

TestPlay::TestPlay() {
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<Role>(TestRole("test_role_0")), std::make_unique<Role>(TestRole("test_role_1")), std::make_unique<Role>(TestRole("test_role_2")),
        std::make_unique<Role>(TestRole("test_role_3")), std::make_unique<Role>(TestRole("test_role_4")), std::make_unique<Role>(TestRole("test_role_5")),
        std::make_unique<Role>(TestRole("test_role_6")), std::make_unique<Role>(TestRole("test_role_7")), std::make_unique<Role>(TestRole("test_role_8")),
        std::make_unique<Role>(TestRole("test_role_9")), std::make_unique<Role>(TestRole("test_role_10"))};
}

bool TestPlay::isValidPlay(world_new::World* world) noexcept { return true; }

uint8_t TestPlay::score(world_new::World* world) noexcept { return 10; }

void TestPlay::assignRoles() noexcept {
    Dealer dealer{world->getWorld().value(), &field};

    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"test_role_0", {closeToBallFlag}});
    flagMap.insert({"test_role_1", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_2", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_3", {closeToBallFlag}});
    flagMap.insert({"test_role_4", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_5", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_6", {closeToBallFlag}});
    flagMap.insert({"test_role_7", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_8", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_9", {closeToBallFlag}});
    flagMap.insert({"test_role_10", {closeToTheirGoalFlag}});

    auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap);

    stpInfos = std::unordered_map<std::string, StpInfo>{};
    for (auto & role : roles) {
        auto roleName{role->getName()};
        if (distribution.find(roleName) != distribution.end()) {
            auto robot = distribution.find(role->getName())->second;

            stpInfos.emplace(roleName, StpInfo{});
            stpInfos[roleName].setRobot(robot);
            stpInfos[roleName].setField(*world->getField());
        }
    }
}

void TestPlay::calculateInfoForPlay() noexcept {
        for (auto & role : roles) {
            auto roleName{role->getName()};
            if (stpInfos.find(roleName) != stpInfos.end()) {
                auto robot = stpInfos[roleName].getRobot().value();
                // TODO calculate additional info
                stpInfos[roleName].setPosition({SHOOT_TO_POSITION, {robot->getId()*0.2, robot->getId()*0.2}});
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getThem()[0]);
                stpInfos[roleName].setBlockDistance(BlockDistance::HALFWAY);
            }
        }
    }


}  // namespace rtt::ai::stp
