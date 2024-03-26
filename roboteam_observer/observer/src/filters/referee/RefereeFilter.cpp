#include "filters/referee/RefereeFilter.h"

void RefereeFilter::process(const std::vector<proto::Referee> &refereePackets) {
    if (refereePackets.empty()) {
        return;
    }
    firstMessageReceived = true;
    auto packets = refereePackets;
    std::sort(packets.begin(), packets.end(), [](const proto::Referee &a, const proto::Referee &b) { return a.packet_timestamp() < b.packet_timestamp(); });
    latestMessage = packets.back();
}

std::optional<proto::Referee> RefereeFilter::getLastRefereeMessage() const {
    if (firstMessageReceived) {
        return latestMessage;
    } else {
        return std::nullopt;
    }
}
