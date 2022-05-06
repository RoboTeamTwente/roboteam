//
// Created by Dawid Kulikowski on 12/12/2021.
//

#ifndef RTT_INTERFACEFIELDSTATESTORE_H
#define RTT_INTERFACEFIELDSTATESTORE_H

#include <mutex>
#include <proto/State.pb.h>

#include <roboteam_utils/AIData.hpp>
#include <roboteam_utils/Teams.hpp>

class InterfaceFieldStateStore {
private:
    mutable std::mutex mtx;
    proto::State state;

    rtt::AIData yellowAIData;
    rtt::AIData blueAIData;

public:
    void setState(proto::State);
    proto::State getState() const;

    void setAIData(const rtt::AIData&, rtt::Team);
    rtt::AIData getAIData(rtt::Team) const;
};


#endif  // RTT_INTERFACEFIELDSTATESTORE_H
