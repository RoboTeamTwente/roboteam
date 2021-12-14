//
// Created by Dawid Kulikowski on 12/12/2021.
//

#ifndef RTT_INTERFACEFIELDSTATESTORE_H
#define RTT_INTERFACEFIELDSTATESTORE_H

#include <mutex>
#include <roboteam_proto/State.pb.h>

class InterfaceFieldStateStore {
private:
    mutable std::mutex mtx;
    proto::State state;

public:
    void setState(proto::State);
    proto::State getState() const;
};


#endif  // RTT_INTERFACEFIELDSTATESTORE_H
