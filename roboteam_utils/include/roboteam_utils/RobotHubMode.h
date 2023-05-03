//
// Created by Martin Miksik on 03/05/2023.
//

#ifndef RTT_ROBOTHUBMODE_H
#define RTT_ROBOTHUBMODE_H

#include <string>
#include <string_view>

namespace rtt {
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

    inline proto::Setting::RobotHubMode modeToProto(RobotHubMode mode) {
        switch (mode) {
            case RobotHubMode::BASESTATION:
                return proto::Setting::BASESTATION;
            case RobotHubMode::SIMULATOR:
                return proto::Setting::SIMULATOR;
            default:
                return proto::Setting::UNKNOWN;
        }
    }

    inline RobotHubMode modeFromProto(proto::Setting::RobotHubMode mode) {
        switch (mode) {
            case proto::Setting::BASESTATION:
                return RobotHubMode::BASESTATION;
            case proto::Setting::SIMULATOR:
                return RobotHubMode::SIMULATOR;
            default:
                return RobotHubMode::UNKNOWN;
        }
    }

}  // namespace rtt

#endif  // RTT_ROBOTHUBMODE_H
