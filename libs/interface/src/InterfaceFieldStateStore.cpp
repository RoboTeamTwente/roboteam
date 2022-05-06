//
// Created by Dawid Kulikowski on 12/12/2021.
//

#include "InterfaceFieldStateStore.h"
void InterfaceFieldStateStore::setState(proto::State state) {
    std::scoped_lock lck(mtx);

    this->state = std::move(state);
}

proto::State InterfaceFieldStateStore::getState() const {
    std::scoped_lock lck(mtx);

    return this->state;
}

rtt::AIData InterfaceFieldStateStore::getAIData(rtt::Team team) const {
    return team == rtt::Team::YELLOW ? this->yellowAIData : this->blueAIData;
}

void InterfaceFieldStateStore::setAIData(const rtt::AIData &data, rtt::Team team) {
    if (team == rtt::Team::YELLOW) {
        this->yellowAIData = data;
    } else if (team == rtt::Team::BLUE) {
        this->blueAIData = data;
    }
}