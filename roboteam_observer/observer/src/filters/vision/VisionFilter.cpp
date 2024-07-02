#include "filters/vision/VisionFilter.h"
proto::World VisionFilter::process(const std::vector<proto::SSL_WrapperPacket>& packets, const std::vector<rtt::RobotsFeedback>& robotData, const std::vector<int>& camera_ids) {
    processGeometry(packets);
    processDetections(packets, robotData, camera_ids);

    // TODO for now not extrapolating because grsim sends packets from 1970...
    Time extroplatedToTime = getExtrapolationTimeForPolicy();

    return worldFilter.getWorldPrediction(extroplatedToTime);
}
void VisionFilter::processGeometry(const std::vector<proto::SSL_WrapperPacket>& packets) {
    for (const auto& packet : packets) {
        if (packet.has_geometry()) {
            geomFilter.process(packet.geometry());
        }
    }
}
void VisionFilter::processDetections(const std::vector<proto::SSL_WrapperPacket>& packets, const std::vector<rtt::RobotsFeedback>& robotData, const std::vector<int>& camera_ids) {
    std::vector<proto::SSL_DetectionFrame> detectionFrames;
    for (const auto& packet : packets) {
        if (packet.has_detection()) {
            detectionFrames.push_back(packet.detection());
            Time detectionTime(packet.detection().t_capture());
            if (detectionTime > lastPacketTime) {
                lastPacketTime = detectionTime;
            }
        }
    }
    worldFilter.process(detectionFrames, robotData, camera_ids, geomFilter);
}
void VisionFilter::updateRobotParameters(const TwoTeamRobotParameters& parameters) { worldFilter.updateRobotParameters(parameters); }
std::optional<proto::SSL_GeometryData> VisionFilter::getGeometry() const {
    if (geomFilter.receivedFirstGeometry()) {
        return geomFilter.getGeometry();
    }
    return std::nullopt;
}

Time VisionFilter::getExtrapolationTimeForPolicy() const {
    switch (extrapolationPolicy) {
        case TimeExtrapolationPolicy::REALTIME:
            return Time::now();
        case TimeExtrapolationPolicy::LAST_RECEIVED_PACKET_TIME:
            return lastPacketTime;
    }
    return lastPacketTime;  // Fallback option
}

void VisionFilter::setExtrapolationPolicy(VisionFilter::TimeExtrapolationPolicy policy) { extrapolationPolicy = policy; }
