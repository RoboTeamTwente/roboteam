//
// Created by rolf on 19-10-20.
//

#ifndef RTT_OBSERVER_H
#define RTT_OBSERVER_H

#include <proto/messages_robocup_ssl_referee.pb.h>
#include <proto/messages_robocup_ssl_wrapper.pb.h>
#include <proto/RobotData.pb.h>
#include <proto/State.pb.h>
#include <roboteam_utils/Time.h>

#include "filters/vision/VisionFilter.h"
#include "filters/referee/RefereeFilter.h"
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
     * @param time time to extrapolate the world to
     * @param visionPackets all of the packets received from vision, including detectionFrames and geometry information
     * @param refereePackets All ofthe packets which were received from the referee.
     *@return The entire known/predicted state of the game at this point in time.
     */
    proto::State process(Time extrapolatedTo,
                 const std::vector<proto::SSL_WrapperPacket>& visionPackets,
                 const std::vector<proto::SSL_Referee>& refereePackets,
                 std::vector<proto::RobotData> robotData);

private:
    RobotParameterDatabase parameterDatabase;
    VisionFilter visionFilter;
    RefereeFilter refereeFilter;

    void updateRobotParams(std::vector<proto::SSL_Referee> refereePackets);


    void updateVision(const std::vector<proto::SSL_WrapperPacket> &visionPackets);

    void updateReferee(const std::vector<proto::SSL_Referee> &refereePackets);
};


#endif //RTT_OBSERVER_H
