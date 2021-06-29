//
// Created by rolf on 19-10-20.
//

#include "Observer.h"

proto::State
Observer::process(Time extrapolatedTo,
                  const std::vector<proto::SSL_WrapperPacket>& visionPackets, const std::vector<proto::SSL_Referee>& refereePackets,
                  std::vector<proto::RobotData> robotData) {

    updateRobotParams(refereePackets);
    proto::State state;
    proto::World world = visionFilter.process(visionPackets,extrapolatedTo);
    updateReferee(refereePackets);



    state.mutable_last_seen_world()->CopyFrom(world);
    state.mutable_command_extrapolated_world()->CopyFrom(world);//TODO; actually do extrapolation
    auto twoTeamParams = parameterDatabase.getParams();

    auto blueParams = twoTeamParams.blueTeamProto();
    auto yellowParams = twoTeamParams.yellowTeamProto();

    state.mutable_blue_robot_parameters()->CopyFrom(blueParams);
    state.mutable_yellow_robot_parameters()->CopyFrom(yellowParams);

    //TODO: define a default geometry so that null geometries no longer exist
    auto geometry = visionFilter.getGeometry();
    if(geometry.has_value()){
        state.mutable_field()->CopyFrom(geometry.value());
    }

    std::optional<proto::SSL_Referee> refMsg = refereeFilter.getLastRefereeMessage();
    if (refMsg){
        state.mutable_referee()->CopyFrom(refMsg.value());
    }

    for(const auto& visionPacket : visionPackets){
        proto::SSL_WrapperPacket * packet = state.add_processed_vision_packets();
        packet->CopyFrom(visionPacket);
    }
    for(const auto& refpacket : refereePackets){
        proto::SSL_Referee * packet = state.add_processed_referee_packets();
        packet->CopyFrom(refpacket);    }
    return state;
}

void Observer::updateRobotParams(std::vector<proto::SSL_Referee> refereePackets) {
    //sort the referee packets; we only use the last one as there is no additional information in between packets
    std::sort(refereePackets.begin(), refereePackets.end(),
              [](const proto::SSL_Referee &a, const proto::SSL_Referee &b) {
                  return a.packet_timestamp() < b.packet_timestamp();
              });
    TwoTeamRobotParameters parameters = !refereePackets.empty() ? parameterDatabase.update(refereePackets.back())
                                                                : parameterDatabase.getParams();
    if (parameters.blueChanged || parameters.yellowChanged) {
        visionFilter.updateRobotParameters(parameters);
    }
}

void Observer::updateReferee(const std::vector<proto::SSL_Referee> &refereePackets) {
    refereeFilter.process(refereePackets);
}

