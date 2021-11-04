//
// Created by rolf on 05-11-20.
//

#ifndef RTT_REFEREEFILTER_H
#define RTT_REFEREEFILTER_H

#include <roboteam_proto/messages_robocup_ssl_referee.pb.h>
#include <roboteam_utils/Time.h>

#include <optional>
class RefereeFilter {
   public:
    void process(const std::vector<proto::SSL_Referee> &refereePackets);
    std::optional<proto::SSL_Referee> getLastRefereeMessage() const;

   private:
    bool firstMessageReceived = false;
    proto::SSL_Referee latestMessage;
};

#endif  // RTT_REFEREEFILTER_H
