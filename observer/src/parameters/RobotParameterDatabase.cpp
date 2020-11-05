//
// Created by rolf on 21-10-20.
//

#include "parameters/RobotParameterDatabase.h"

TwoTeamRobotParameters RobotParameterDatabase::update(const proto::SSL_Referee &refMessage) {
    TwoTeamRobotParameters twoTeamParameters;
    if (refMessage.blue().name() != blueName){
        twoTeamParameters.blueChanged = true;
        blueName = refMessage.blue().name();
        blueParameters = getTeamParameters(refMessage.blue().name());
    }

    if (refMessage.yellow().name() != yellowName){
        twoTeamParameters.yellowChanged = true;
        yellowName = refMessage.yellow().name();
        yellowParameters = getTeamParameters(refMessage.yellow().name());
    }
    twoTeamParameters.blueParameters = blueParameters;
    twoTeamParameters.yellowParameters = yellowParameters;

    return twoTeamParameters;
}

TwoTeamRobotParameters RobotParameterDatabase::getParams() const {
    TwoTeamRobotParameters twoTeamParameters;
    twoTeamParameters.blueParameters = blueParameters;
    twoTeamParameters.yellowParameters = yellowParameters;
    return twoTeamParameters;
}

RobotParameters RobotParameterDatabase::getTeamParameters(const std::string &teamName) {
    //These teamnames should be the same as set in
    //https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/src/components/settings/team/TeamName.vue
    //TODO: add teams we play against here.
    if(teamName == "RoboTeam Twente"){
        return RobotParameters::RTT_2020();
    }else{
        return RobotParameters::DEFAULT();
    }
}

proto::TeamParameters TwoTeamRobotParameters::blueTeamProto() const {
    proto::TeamParameters teamParams;
    teamParams.set_didchange(blueChanged);
    auto params = blueParameters.toProto();
    teamParams.mutable_parameters()->CopyFrom(params);
    return teamParams;
}

proto::TeamParameters TwoTeamRobotParameters::yellowTeamProto() const {
    proto::TeamParameters teamParams;
    teamParams.set_didchange(yellowChanged);
    auto params = yellowParameters.toProto();
    teamParams.mutable_parameters()->CopyFrom(params);
    return teamParams;
}
