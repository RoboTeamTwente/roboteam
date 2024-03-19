//
// Created by rolf on 05-11-20.
//

#ifndef RTT_REFEREEFILTER_H
#define RTT_REFEREEFILTER_H

#include <proto/messages_robocup_ssl_referee.pb.h>
#include <roboteam_utils/Time.h>

#include <optional>

/*
 * @brief A class which filters the referee information into one coherent message.
 * For now it just returns the latest referee message
 */
class RefereeFilter {
   public:
    /**
     * @param refereePackets packets to update the filter with
     */
    void process(const std::vector<proto::Referee> &refereePackets);
    /**
     * Returns the latest referee message if it exists.
     */
    std::optional<proto::Referee> getLastRefereeMessage() const;

   private:
    bool firstMessageReceived = false;
    proto::Referee latestMessage;
};

#endif  // RTT_REFEREEFILTER_H
