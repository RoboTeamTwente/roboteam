//
// Created by rolf on 21-10-20.
//

#ifndef RTT_ROBOTPARAMETERDATABASE_H
#define RTT_ROBOTPARAMETERDATABASE_H
#include <roboteam_proto/messages_robocup_ssl_referee.pb.h>
#include "observer/data/RobotParameters.h"
struct TwoTeamRobotParameters{
    bool blueChanged = false;
    bool yellowChanged = false;
    RobotParameters blueParameters;
    RobotParameters yellowParameters;
    [[nodiscard]] proto::TeamParameters yellowTeamProto() const;
    [[nodiscard]] proto::TeamParameters blueTeamProto() const;
};
class RobotParameterDatabase {
public:
    TwoTeamRobotParameters update(const proto::SSL_Referee& refMessage);
    [[nodiscard]] TwoTeamRobotParameters getParams() const;

    static RobotParameters getTeamParameters(const std::string& teamName);
private:
    std::string blueName;
    RobotParameters blueParameters;
    std::string yellowName;
    RobotParameters yellowParameters;
};


#endif //RTT_ROBOTPARAMETERDATABASE_H
