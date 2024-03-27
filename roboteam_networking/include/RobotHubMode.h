#ifndef RTT_ROBOTHUBMODE_H
#define RTT_ROBOTHUBMODE_H

#include <proto/GameSettings.pb.h>

#include <string>
#include <string_view>

namespace rtt::net {
/**
 * Enumerator that tells where RobotHub should send the robot commands
 */
enum RobotHubMode { UNKNOWN, SIMULATOR, BASESTATION };

inline std::string_view robotHubModeToString(RobotHubMode mode) {
    switch (mode) {
        case RobotHubMode::BASESTATION:
            return "Basestation";
        case RobotHubMode::SIMULATOR:
            return "Simulator";
        default:
            return "Unknown";
    }
}

inline proto::GameSettings::RobotHubMode robotHubModeToProto(RobotHubMode mode) {
    switch (mode) {
        case RobotHubMode::BASESTATION:
            return proto::GameSettings::BASESTATION;
        case RobotHubMode::SIMULATOR:
            return proto::GameSettings::SIMULATOR;
        default:
            return proto::GameSettings::UNKNOWN;
    }
}

inline RobotHubMode robotHubModeFromProto(proto::GameSettings::RobotHubMode mode) {
    switch (mode) {
        case proto::GameSettings::BASESTATION:
            return RobotHubMode::BASESTATION;
        case proto::GameSettings::SIMULATOR:
            return RobotHubMode::SIMULATOR;
        default:
            return RobotHubMode::UNKNOWN;
    }
}

}  // namespace rtt::net

#endif  // RTT_ROBOTHUBMODE_H
