//
// Created by rolf on 23-06-21.
//

#include "filters/vision/VisionFilter.h"
proto::World VisionFilter::process(const std::vector<proto::SSL_WrapperPacket> &packets,
                                   const std::vector<rtt::RobotsFeedback>& robotData) {
  bool geometry_updated = processGeometry(packets);
  processDetections(packets,geometry_updated,robotData);

  //TODO for now not extrapolating because grsim sends packets from 1970...
  Time extroplatedToTime = getExtrapolationTimeForPolicy();

  return worldFilter.getWorldPrediction(extroplatedToTime);
}
bool VisionFilter::processGeometry(const std::vector<proto::SSL_WrapperPacket>& packets) {
  bool newGeometry = false;
  for(const auto& packet : packets){
    if(packet.has_geometry()){
      if(geomFilter.process(packet.geometry())){
        newGeometry = true;
      }
    }
  }
  return newGeometry;
}
void VisionFilter::processDetections(const std::vector<proto::SSL_WrapperPacket> &packets, bool update_geometry,
                                     const std::vector<rtt::RobotsFeedback>& robotData) {
  if(update_geometry){
    worldFilter.updateGeometry(geomFilter.getGeometry());
  }
  std::vector<proto::SSL_DetectionFrame> detectionFrames;
  for (const auto& packet : packets) {
    if(packet.has_detection()){
      detectionFrames.push_back(packet.detection());
      Time detectionTime(packet.detection().t_capture());
      if (detectionTime > lastPacketTime){
        lastPacketTime = detectionTime;
      }
    }
  }
  worldFilter.process(detectionFrames,robotData);
}
void VisionFilter::updateRobotParameters(const TwoTeamRobotParameters &parameters) {
  worldFilter.updateRobotParameters(parameters);
}
std::optional<proto::SSL_GeometryData> VisionFilter::getGeometry() const {
  if(geomFilter.receivedFirstGeometry()){
    return geomFilter.getGeometry();
  }
  return std::nullopt;
}

Time VisionFilter::getExtrapolationTimeForPolicy() const {
    switch(extrapolationPolicy){
        case TimeExtrapolationPolicy::REALTIME:
            return Time::now();
        case TimeExtrapolationPolicy::LAST_RECEIVED_PACKET_TIME:
            return lastPacketTime;
    }
    return lastPacketTime; //Fallback option
}

void VisionFilter::setExtrapolationPolicy(VisionFilter::TimeExtrapolationPolicy policy) {
    extrapolationPolicy = policy;
}
