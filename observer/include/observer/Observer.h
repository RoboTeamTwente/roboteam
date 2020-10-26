//
// Created by rolf on 19-10-20.
//

#ifndef RTT_OBSERVER_H
#define RTT_OBSERVER_H

#include <roboteam_proto/messages_robocup_ssl_referee.pb.h>
#include <roboteam_proto/messages_robocup_ssl_wrapper.pb.h>
#include <roboteam_proto/RobotData.pb.h>
#include <roboteam_proto/State.pb.h>
#include <roboteam_utils/Time.h>

#include "filters/WorldFilter.h"
#include "filters/geometry/GeometryFilter.h"
#include "parameters/RobotParameterDatabase.h"

/**
 * @brief This class provides a unified interface for processing all the real-time information provided by the SSL and our system
 * on the physical state and the referee's current state of the game.
 * Note this interface is NOT responsible for networking-related issues, e.g. all objects passed to it are assumed to be from a
 * trusted source. Some packets may however still be discarded if they have bad timestamps for example.
 * @author Rolf
 */
class Observer {
public:
    /**
     * Calls all of the feedbacks for processing relevant data given the given input data.
     * @param visionPackets all of the packets received from vision, including detectionFrames and geometry information
     * @param refereePackets All ofthe packets which were received from the referee.
     *
     */
    void process(std::vector<proto::SSL_WrapperPacket> visionPackets,
                 std::vector<proto::SSL_Referee> refereePackets,
                 std::vector<proto::RobotData> robotData);

    /**
     * Gets (predicts) state of the game at a point slightly in the future.
     * @param time
     * @return The entire known/predicted state of the game at this point in time.
     */
    [[nodiscard]] proto::State getState(Time time) const;
private:
    RobotParameterDatabase parameterDatabase;
    WorldFilter worldFilter;
    GeometryFilter geometryFilter;
    //RefereeFilter refereeFilter;

};


#endif //RTT_OBSERVER_H
