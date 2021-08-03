//
// Created by rolf on 05-11-20.
//

#include "filters/referee/RefereeFilter.h"

void RefereeFilter::process(const std::vector<proto::SSL_Referee> &refereePackets) {
    if(refereePackets.empty()){
        return;
    }
    firstMessageReceived = true;
    auto packets = refereePackets;
    std::sort(packets.begin(), packets.end(),
              [](const proto::SSL_Referee &a, const proto::SSL_Referee &b) {
                  return a.packet_timestamp() < b.packet_timestamp();
              });
    latestMessage = packets.back();
}

std::optional<proto::SSL_Referee> RefereeFilter::getLastRefereeMessage() const {
    if(firstMessageReceived){
        return latestMessage;
    }else{
        return std::nullopt;
    }
}
