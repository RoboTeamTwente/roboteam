//
// Created by rolf on 19-10-20.
//
#include <optional>

#include "Observer.h"

proto::State Observer::process(Time extrapolatedTo, const std::vector<proto::SSL_WrapperPacket> &visionPackets, const std::vector<proto::SSL_Referee> &refereePackets,
                               std::vector<proto::RobotData> robotData) {
    updateRobotParams(refereePackets);
    updateGeometry(visionPackets);
    updateWorld(visionPackets);
    updateReferee(refereePackets);

    proto::State state;
    proto::World world = worldFilter.getWorld();
    state.mutable_last_seen_world()->CopyFrom(world);
    state.mutable_command_extrapolated_world()->CopyFrom(world);  // TODO; actually do extrapolation
    auto twoTeamParams = parameterDatabase.getParams();

    auto blueParams = twoTeamParams.blueTeamProto();
    auto yellowParams = twoTeamParams.yellowTeamProto();

    state.mutable_blue_robot_parameters()->CopyFrom(blueParams);
    state.mutable_yellow_robot_parameters()->CopyFrom(yellowParams);

    // TODO: define a default geometry
    if (geometryFilter.receivedFirstGeometry()) {
        auto geometry = geometryFilter.getGeometry();
        state.mutable_field()->CopyFrom(geometry);
    }

    std::optional<proto::SSL_Referee> refMsg = refereeFilter.getLastRefereeMessage();
    if (refMsg) {
        state.mutable_referee()->CopyFrom(refMsg.value());
    }

    for (const auto &visionPacket : visionPackets) {
        proto::SSL_WrapperPacket *packet = state.add_processed_vision_packets();
        packet->CopyFrom(visionPacket);
    }
    for (const auto &refpacket : refereePackets) {
        proto::SSL_Referee *packet = state.add_processed_referee_packets();
        packet->CopyFrom(refpacket);
    }
    return state;
}

void Observer::updateWorld(const std::vector<proto::SSL_WrapperPacket> &visionPackets) {
    std::vector<proto::SSL_DetectionFrame> detectionFrames;
    for (const auto &packet : visionPackets) {
        if (packet.has_detection()) {
            detectionFrames.push_back(packet.detection());
        }
    }
    worldFilter.process(detectionFrames);
}

void Observer::updateGeometry(const std::vector<proto::SSL_WrapperPacket> &visionPackets) {
    bool geometryWasUpdated = false;
    for (const auto &packet : visionPackets) {
        if (packet.has_geometry()) {
            geometryWasUpdated |= geometryFilter.process(packet.geometry());
        }
    }
    if (geometryWasUpdated) {
        worldFilter.setGeometry(geometryFilter.getGeometry());
    }
}

void Observer::updateRobotParams(std::vector<proto::SSL_Referee> refereePackets) {
    // sort the referee packets; we only use the last one as there is no additional information in between packets
    std::sort(refereePackets.begin(), refereePackets.end(), [](const proto::SSL_Referee &a, const proto::SSL_Referee &b) { return a.packet_timestamp() < b.packet_timestamp(); });
    TwoTeamRobotParameters parameters = !refereePackets.empty() ? parameterDatabase.update(refereePackets.back()) : parameterDatabase.getParams();
    if (parameters.blueChanged || parameters.yellowChanged) {
        worldFilter.setRobotParameters(parameters);
    }
}

void Observer::updateReferee(const std::vector<proto::SSL_Referee> &refereePackets) { refereeFilter.process(refereePackets); }
