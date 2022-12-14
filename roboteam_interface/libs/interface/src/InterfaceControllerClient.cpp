//
// Created by Dawid Kulikowski on 06/01/2022.
//

#include "InterfaceControllerClient.h"
namespace rtt::Interface {
    proto::UiValues InterfaceControllerClient::getDataForRemote() const noexcept {
        return this->vals->toProto();
    }

    void InterfaceControllerClient::handleData(const proto::ModuleState& state) {
        if (state.handshakes_size() != 1) {
            return;
        }

        const auto& handshake = state.handshakes(0);

        this->decls->handleData(handshake.declarations());
        this->vals->handleData(handshake.values(), decls, InterfaceSettingsPrecedence::IFACE);
    }

    std::weak_ptr<MessageCache<proto::State>> InterfaceControllerClient::getFieldState() const {
        return fieldState;
    }

    void InterfaceControllerClient::field_state_callback(const std::string& state) {
        this->fieldState->setMessage(state);
    }

    void InterfaceControllerClient::stop() {
        this->field_subscriber.reset(); // Prevent ugly crashes on exit
        this->blueDataSub.reset();
        this->yellowDataSub.reset();

        InterfaceController::stop();
    }

    void InterfaceControllerClient::loop() {
        while (this->shouldRun) {
            this->updateCounter = (this->updateCounter + 1) % MAX_CYCLES_WITHOUT_UPDATE;

            if (this->updateCounter == 0  || this->updateMarker) {
                bool t = true;
                updateMarker.compare_exchange_strong(t, false);
                loop_iter();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_ATTEMPT_MS));
        }
    }

    void InterfaceControllerClient::markForUpdate() {
        this->updateMarker.store(true);
    }

    std::map<rtt::Team, std::weak_ptr<MessageCache<proto::AIData>>> InterfaceControllerClient::getPaths() {
        return {{rtt::Team::YELLOW, this->aiState.at(rtt::Team::YELLOW)}, {rtt::Team::BLUE, this->aiState.at(rtt::Team::BLUE)}};
    }
}
