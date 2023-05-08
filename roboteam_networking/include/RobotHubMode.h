//
// Created by Martin Miksik on 03/05/2023.
//

#ifndef RTT_ROBOTHUBMODE_H
#define RTT_ROBOTHUBMODE_H

#include <string>
#include <string_view>
#include <proto/GameSettings.pb.h>

namespace rtt::net {
    /**
     * Enumerator that tells where RobotHub should send the robot commands
     */
    enum RobotHubMode { UNKNOWN,
                        SIMULATOR,
                        BASESTATION };

    inline std::string_view modeToString(RobotHubMode mode) {
        switch (mode) {
            case RobotHubMode::BASESTATION:
                return "Basestation";
            case RobotHubMode::SIMULATOR:
                return "Simulator";
            default:
                return "Unknown";
        }
    }

    inline proto::GameSettings::RobotHubMode modeToProto(RobotHubMode mode) {
        switch (mode) {
            case RobotHubMode::BASESTATION:
                return proto::GameSettings::BASESTATION;
            case RobotHubMode::SIMULATOR:
                return proto::GameSettings::SIMULATOR;
            default:
                return proto::GameSettings::UNKNOWN;
        }
    }

    inline RobotHubMode modeFromProto(proto::GameSettings::RobotHubMode mode) {
        switch (mode) {
            case proto::GameSettings::BASESTATION:
                return RobotHubMode::BASESTATION;
            case proto::GameSettings::SIMULATOR:
                return RobotHubMode::SIMULATOR;
            default:
                return RobotHubMode::UNKNOWN;
        }
    }

}  // namespace rtt

#endif  // RTT_ROBOTHUBMODE_H
