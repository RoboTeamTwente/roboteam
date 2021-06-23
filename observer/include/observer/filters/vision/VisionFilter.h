//
// Created by rolf on 23-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_VISIONFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_VISIONFILTER_H_
#include "GeometryFilter.h"
#include "WorldFilter.h"
#include "roboteam_utils/Time.h"
#include "roboteam_proto/messages_robocup_ssl_wrapper.pb.h"

/**
 * @author Rolf
 * @class VisionFilter Filters and processes all information received from SSL-Vision into coherent information.
 *  This primarily involves filters to track the robots and the ball and estimate their position and speed, and some processing of the geometry
 *  At some point in the future, it is also intended that robot command and feedback information is processed and filtered.
 */
class VisionFilter {
 public:
  /**
   *  This function updates the vision filter with vision packets and returns the world state as estimated at time.
   *  Note time needs to be at some point in the future!
   * @param packets the received packets to update visionfilter with
   * @param time the wanted time to estimate the world state at (can extrapolate in the future)
   * @return a world state, extrapolated to the given time
   */
  proto::World process(const std::vector<proto::SSL_WrapperPacket>& packets, Time time);

 private:

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
  void processDetections(const std::vector<proto::SSL_WrapperPacket>& packets, bool update_geometry);
  GeometryFilter geomFilter;
  WorldFilter worldFilter;
  Time lastPacketTime;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_VISIONFILTER_H_
