//
// Created by Dawid Kulikowski on 12/12/2021.
//

#ifndef RTT_INTERFACEFIELDSTATESTORE_H
#define RTT_INTERFACEFIELDSTATESTORE_H

#include <mutex>
#include <proto/State.pb.h>

class InterfaceFieldStateStore {
private:
    mutable std::mutex mtx;

    proto::State cachedState;
    std::optional<std::string> state;

public:
    void setState(proto::State);
    void setState(std::string);
    std::optional<proto::State> getState();
};


#endif  // RTT_INTERFACEFIELDSTATESTORE_H
