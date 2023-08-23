#include <RobotFeedbackNetworker.hpp>
#include <array>
#include <chrono>
#include <mutex>
#include <utility>

using namespace rtt;

constexpr int MAX_AMOUNT_OF_ROBOTS = 16;
constexpr int FORGET_TIME_MS = 1500;  // After this time, we forget if a robot had that ball or not

typedef std::array<std::pair<rtt::RobotFeedback, std::chrono::steady_clock::time_point>, MAX_AMOUNT_OF_ROBOTS> RobotStatuses;

std::mutex yellowProtector;
RobotStatuses yellowBots;
std::mutex blueProtector;
RobotStatuses blueBots;

constexpr char HAS_BAL_CHAR = 'O';
constexpr char HAS_NOT_BAL_CHAR = 'X';
constexpr char BROKEN_SENSOR_CHAR = '*';

auto lastTimePrintHappened = std::chrono::steady_clock::now();
bool updateHappenedSincePrint = false;

std::ostream& operator<<(std::ostream& os, const std::pair<rtt::RobotFeedback, std::chrono::steady_clock::time_point>& data) {
    auto now = std::chrono::steady_clock::now();
    auto timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(now - data.second).count();

    if (timeSinceLastUpdate > FORGET_TIME_MS) {
        // Just print an empty robot status
        os << "   ";
    } else {
        if (data.first.ballSensorIsWorking) {
            os << (data.first.ballSensorSeesBall ? HAS_BAL_CHAR : HAS_NOT_BAL_CHAR);
        } else {
            os << BROKEN_SENSOR_CHAR;
        }

        os << "/";

        os << (data.first.dribblerSeesBall ? HAS_BAL_CHAR : HAS_NOT_BAL_CHAR);
    }


    return os;
}


void printStatus() {
    const auto& y = yellowBots;
    const auto& b = blueBots;

    std::scoped_lock<std::mutex, std::mutex> locks(yellowProtector, blueProtector);

    std::cout << "┏━━━┫ Yellow ┣━━━┳━━━━┫ Blue ┣━━━━┓" << std::endl
              << "┃ 0: " << y[0] << "  8: " << y[8] << " ┃ 0: " << b[0] << "  8: " << b[8] << " ┃ A/B:" << std::endl
              << "┃ 1: " << y[1] << "  9: " << y[9] << " ┃ 1: " << b[1] << "  9: " << b[9] << " ┃ A: BallSensor status" << std::endl
              << "┃ 2: " << y[2] << " 10: " << y[10] << " ┃ 2: " << b[2] << " 10: " << b[10] << " ┃ B: Dribbler status" << std::endl
              << "┃ 3: " << y[3] << " 11: " << y[11] << " ┃ 3: " << b[3] << " 11: " << b[11] << " ┃" << std::endl
              << "┃ 4: " << y[4] << " 12: " << y[12] << " ┃ 4: " << b[4] << " 12: " << b[12] << " ┃ O: Notices ball" << std::endl
              << "┃ 5: " << y[5] << " 13: " << y[13] << " ┃ 5: " << b[5] << " 13: " << b[13] << " ┃ X: Does not notice ball" << std::endl
              << "┃ 6: " << y[6] << " 14: " << y[14] << " ┃ 6: " << b[6] << " 14: " << b[14] << " ┃ *: Broken" << std::endl
              << "┃ 7: " << y[7] << " 15: " << y[15] << " ┃ 7: " << b[7] << " 15: " << b[15] << " ┃" << std::endl
              << "┗━━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━━━┛" << std::endl;

    lastTimePrintHappened = std::chrono::steady_clock::now();
}

void onFeedback(const rtt::RobotsFeedback& feedbacks) {
    auto& arr = feedbacks.team == Team::YELLOW ? yellowBots : blueBots;
    std::scoped_lock<std::mutex> lock(feedbacks.team == Team::YELLOW ? yellowProtector : blueProtector);

    int updateCounter = 0;

    auto now = std::chrono::steady_clock::now();
    for (const auto& feedback : feedbacks.feedback) {
        auto id = feedback.id;
        if (id >= 0 && id < MAX_AMOUNT_OF_ROBOTS) {
            // If the feedback is different than what we previously had, update it
            if (feedback != arr[id].first) {
                arr[id].first = feedback;
                updateCounter++;
            }
            // Always update the timer if we received data about this robot.
            arr[feedback.id].second = now;
        }
    }

    if (updateCounter > 0) updateHappenedSincePrint = true;
}

int main() {
    auto feedbackSub = net::RobotFeedbackSubscriber(onFeedback);

    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto timeSinceLastPrint = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimePrintHappened).count();

        if (updateHappenedSincePrint || timeSinceLastPrint > 1000) {
            printStatus();
            updateHappenedSincePrint = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
