//
// Created by rolf on 23-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_VISIONFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_VISIONFILTER_H_
#include "GeometryFilter.h"
#include "WorldFilter.h"
#include "roboteam_utils/Time.h"
#include "proto/messages_robocup_ssl_wrapper.pb.h"
#include "roboteam_utils/RobotFeedback.hpp"

/**
 * @author Rolf
 * @class VisionFilter Filters and processes all information received from SSL-Vision into coherent information.
 *  This primarily involves filters to track the robots and the ball and estimate their position and speed, and some processing of the geometry
 *  Additionally, it also filters feedback information from the robots into a coherent picture.
 *  At some point in the future, it is also intended that sent robots commands are processed.
 */
class VisionFilter {
 public:
  /**
   *  This function updates the vision filter with vision packets and returns the world state as estimated at time.
   *  Note time needs to be at some point in the future!
   * @param packets the received packets to update visionfilter with
   * @param time the wanted time to estimate the world state at (can extrapolate in the future)
   * @param feedback the feedback packets received for all robots
   * @return a world state, extrapolated to the given time
   */
  proto::World process(const std::vector<proto::SSL_WrapperPacket>& packets,
                       const std::vector<rtt::RobotsFeedback>& feedback);

  /*
   * Updates the robot definitions the vision filter uses in world prediction
   */
  void updateRobotParameters(const TwoTeamRobotParameters& parameters);

  std::optional<proto::SSL_GeometryData> getGeometry() const;


  enum class TimeExtrapolationPolicy{
      REALTIME,
      LAST_RECEIVED_PACKET_TIME
  };

  void setExtrapolationPolicy(TimeExtrapolationPolicy policy);
 private:

    Time getExtrapolationTimeForPolicy() const;
  /**
   * processes any geometry data and passes it to the
   * @param packets the packets to check for geometry changes
   * @return True if the geometry was somehow changed/updated and different
   */
  bool processGeometry(const std::vector<proto::SSL_WrapperPacket>& packets);
  /**
   * @param packets the relevant detection frames from SSL-vision
   * @param update_geometry Set this to true to update the Geometry used by the world filter
   */
  void processDetections(const std::vector<proto::SSL_WrapperPacket>& packets, bool update_geometry,
                         const std::vector<rtt::RobotsFeedback>& robotData);

  GeometryFilter geomFilter;
  WorldFilter worldFilter;
  Time lastPacketTime;
  TimeExtrapolationPolicy extrapolationPolicy = TimeExtrapolationPolicy::LAST_RECEIVED_PACKET_TIME;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_VISIONFILTER_H_
