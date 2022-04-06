//
// Created by Dawid Kulikowski on 06/01/2022.
//

#include "InterfaceControllerClient.h"
namespace rtt::Interface {
    proto::UiValues InterfaceControllerClient::getDataForRemote(bool) const noexcept {
        return this->vals->toProto();
    }

    void InterfaceControllerClient::handleData(const proto::ModuleState& state) {
        if (state.handshakes_size() != 1) {
            return;
        }

        const auto& handshake = state.handshakes(0);

        this->decls->handleData(handshake.declarations());
        this->vals->handleData(handshake.values(), decls);
//        this->fieldState->setState(state.system_state());
    }

    std::weak_ptr<InterfaceFieldStateStore> InterfaceControllerClient::getFieldState() const {
        return fieldState;
    }

    bool InterfaceControllerClient::hasPriorityData() const noexcept {
        return this->vals->getDidChange();
    }
    void InterfaceControllerClient::field_state_callback(const proto::State& state) {
        this->fieldState->setState(state);
    }
}
