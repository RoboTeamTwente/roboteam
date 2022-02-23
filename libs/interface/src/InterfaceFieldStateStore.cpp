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


