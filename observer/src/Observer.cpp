//
// Created by rolf on 19-10-20.
//

#include "Observer.h"

//TODO: actually implement
proto::State Observer::getState(Time time) const {
    return proto::State();
}

void
Observer::process(std::vector<proto::SSL_WrapperPacket> visionPackets, std::vector<proto::SSL_Referee> refereePackets,
                  std::vector<proto::RobotData> robotData) {
    //sort the referee packets; we only use the last one as there is no additional information in between packets
    std::sort(refereePackets.begin(), refereePackets.end(),
              [](const proto::SSL_Referee &a, const proto::SSL_Referee &b) {
                  return a.packet_timestamp() < b.packet_timestamp();
              });
    TwoTeamRobotParameters parameters = !refereePackets.empty() ? parameterDatabase.update(refereePackets.back())
                                                                : parameterDatabase.update();
    //TODO: add geometry filter
    //TODO: make world filter use geometry and robot parameters for better predictions
}
