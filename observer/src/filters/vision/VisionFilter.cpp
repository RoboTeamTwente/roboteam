//
// Created by rolf on 23-06-21.
//

#include "filters/vision/VisionFilter.h"
proto::World VisionFilter::process(const std::vector<proto::SSL_WrapperPacket> &packets, Time time) {
  bool geometry_updated = processGeometry(packets);
  processDetections(packets,geometry_updated);

  return worldFilter.getWorldPrediction(time);
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
void VisionFilter::processDetections(const std::vector<proto::SSL_WrapperPacket> &packets, bool update_geometry) {
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
  worldFilter.process(detectionFrames);
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
