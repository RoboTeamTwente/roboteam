#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>

#include <cmath>
#include <roboteam_utils/AIData.hpp>
#include <sstream>
using namespace rtt;

std::string getRandomString() {
    constexpr std::string_view CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    std::stringstream ss;
    for (int i = 0; i < SimpleRandom::getInt(0, 10); i++) {
        ss << *SimpleRandom::getRandomElement(CHARS.begin(), CHARS.end());
    }

    return ss.str();
}

RobotSTP getRandomSTP() {
    RobotSTP randomSTP;
    randomSTP.robotId = SimpleRandom::getInt(0, 15);
    randomSTP.role = getRandomString();
    randomSTP.roleStatus = getRandomString();
    randomSTP.tactic = getRandomString();
    randomSTP.tacticStatus = getRandomString();
    randomSTP.skill = getRandomString();
    randomSTP.skillStatus = getRandomString();
    return randomSTP;
}

RobotPath getRandomPath() {
    RobotPath randomPath;
    randomPath.robotId = SimpleRandom::getInt(0, 15);

    for (int i = 0; i < SimpleRandom::getInt(1, 10); i++) {
        randomPath.points.push_back({SimpleRandom::getDouble(-10, 10), SimpleRandom::getDouble(-10, 10)});
    }
    return randomPath;
}

AIData getRandomAIData() {
    AIData data;
    for (int i = 0; i < SimpleRandom::getInt(0, 15); i++) {
        data.robotStps.push_back(getRandomSTP());
    }
    for (int i = 0; i < SimpleRandom::getInt(0, 15); i++) {
        data.robotPaths.push_back(getRandomPath());
    }
    return data;
}

TEST(AIDataTest, robotPathInstantiation) {
    RobotPath path;

    ASSERT_EQ(path.robotId, 0);
    ASSERT_TRUE(path.points.empty());
}

TEST(AIDataTest, robotSTPInstantiation) {
    RobotSTP stp;

    ASSERT_EQ(stp.robotId, 0);
    ASSERT_TRUE(stp.role == "");
    ASSERT_TRUE(stp.roleStatus == "");
    ASSERT_TRUE(stp.tactic == "");
    ASSERT_TRUE(stp.tacticStatus == "");
    ASSERT_TRUE(stp.skill == "");
    ASSERT_TRUE(stp.skillStatus == "");
}

TEST(AIDataTest, aiDataInstantiation) {
    AIData data;

    ASSERT_TRUE(data.robotPaths.empty());
    ASSERT_TRUE(data.robotStps.empty());
}

TEST(AiDataTest, pathEquals) {
    for (int i = 0; i < 50; i++) {
        auto randomPath = getRandomPath();
        auto copy = randomPath;
        ASSERT_EQ(randomPath, copy);
    }
}

TEST(AiDataTest, stpEquals) {
    for (int i = 0; i < 50; i++) {
        auto randomSTP = getRandomSTP();
        auto copy = randomSTP;
        ASSERT_EQ(randomSTP, copy);
    }
}

TEST(AiDataTest, aiDataEquals) {
    for (int i = 0; i < 50; i++) {
        auto randomAIData = getRandomAIData();
        auto copy = randomAIData;
        ASSERT_EQ(randomAIData, copy);
    }
}