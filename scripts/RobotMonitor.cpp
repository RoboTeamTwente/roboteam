#include <RobotFeedbackNetworker.hpp>

#include <array>
#include <utility>
#include <chrono>
#include <mutex>

using namespace rtt;

enum OwnBallType {
    UNKNOWN, TRUE, FALSE
};

constexpr int MAX_AMOUNT_OF_ROBOTS = 16;
constexpr int FORGET_TIME_MS = 1500; // After this time, we forget if a robot had that ball or not

typedef std::array<std::pair<OwnBallType, std::chrono::steady_clock::time_point>, MAX_AMOUNT_OF_ROBOTS> RobotsWithBall;

std::mutex yellowProtector;
RobotsWithBall yellowBots;
std::mutex blueProtector;
RobotsWithBall blueBots;

auto lastPrintTime = std::chrono::steady_clock::now();
bool updateHappened = false;

std::ostream& operator<<(std::ostream& os, const std::pair<OwnBallType, std::chrono::steady_clock::time_point>& data) {
    switch (data.first) {
        case OwnBallType::FALSE:
            return os << "X";
        case OwnBallType::TRUE:
            return os << "O";
        case OwnBallType::UNKNOWN: default:
            return os << " ";
    }
}

void printStatus() {
    const auto& y = yellowBots;
    const auto& b = blueBots;

    std::scoped_lock<std::mutex, std::mutex> locks(yellowProtector, blueProtector);

    std::cout << "┏━━━━━━━┫ Yellow ┣━━━━━━┳━━━━━━━━┫ Blue ┣━━━━━━━┓" << std::endl
    << "┃ 0: "<<y[0]<<" 4: "<<y[4]<< "  8: "<< y[8]<<" 12: "<<y[12]<<" ┃ 0: "<<b[0]<<" 4: "<<b[4]<<"  8: "<< b[8]<<" 12: "<<b[12]<<" ┃"<<std::endl
    << "┃ 1: "<<y[1]<<" 5: "<<y[5]<< "  9: "<< y[9]<<" 13: "<<y[13]<<" ┃ 1: "<<b[1]<<" 5: "<<b[5]<<"  9: "<< b[9]<<" 13: "<<b[13]<<" ┃"<<std::endl
    << "┃ 2: "<<y[2]<<" 6: "<<y[6]<< " 10: "<<y[10]<<" 14: "<<y[14]<<" ┃ 2: "<<b[2]<<" 6: "<<b[6]<<" 10: "<<b[10]<<" 14: "<<b[14]<<" ┃"<<std::endl
    << "┃ 3: "<<y[3]<<" 7: "<<y[7]<< " 11: "<<y[11]<<" 15: "<<y[15]<<" ┃ 3: "<<b[3]<<" 7: "<<b[7]<<" 11: "<<b[11]<<" 15: "<<b[15]<<" ┃"<<std::endl
    << "┗━━━━━━━━━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl
    << "O: Bot has ball X: Bot doesn't have ball ' ': Unknown" << std::endl;

    lastPrintTime = std::chrono::steady_clock::now();
}

// Initializes the array to an empty state
void clearArray(RobotsWithBall& arr) {
    auto now = std::chrono::steady_clock::now();
    for (int i = 0; i < MAX_AMOUNT_OF_ROBOTS; i++) {
        arr[i] = { OwnBallType::UNKNOWN, now };
    }
}

// Removes outdated data of the array
int cleanUpArray(RobotsWithBall& arr) {
    auto now = std::chrono::steady_clock::now();
    int cleanCounter = 0;

    for (auto &[type, time] : arr) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - time).count();
        if (type != OwnBallType::UNKNOWN && duration > FORGET_TIME_MS) {
            type = OwnBallType::UNKNOWN;
            cleanCounter++;
        }
    }
    return cleanCounter;
}

void onFeedback(const rtt::RobotsFeedback& feedbacks) {
    auto& arr = feedbacks.team == Team::YELLOW ? yellowBots : blueBots;
    std::scoped_lock<std::mutex> lock(feedbacks.team == Team::YELLOW ? yellowProtector : blueProtector);

    int updateCounter = 0;

    auto now = std::chrono::steady_clock::now();
    for (const auto& feedback : feedbacks.feedback) {
        if (feedback.id >= 0 && feedback.id < MAX_AMOUNT_OF_ROBOTS) {
            auto type = feedback.hasBall ? OwnBallType::TRUE : OwnBallType::FALSE;

            if (type != arr[feedback.id].first) updateCounter++;

            arr[feedback.id] = { type, now };
        }
    }

    if (updateCounter > 0) updateHappened = true;
}

int main() {
    clearArray(yellowBots);
    clearArray(blueBots);

    auto feedbackSub = net::RobotFeedbackSubscriber(onFeedback);

    while (true) {
        if (cleanUpArray(yellowBots) > 0 || cleanUpArray(blueBots) > 0) updateHappened = true;

        auto now = std::chrono::steady_clock::now();
        auto timeSinceLastPrint = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPrintTime).count();

        if (updateHappened || timeSinceLastPrint > 1000) {
            printStatus();
            updateHappened = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}

