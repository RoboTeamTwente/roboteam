//
// Created by Dawid Kulikowski on 06/01/2022.
//

#include "InterfaceControllerClient.h"
namespace rtt::Interface {
    InterfaceControllerClient::InterfaceControllerClient()
         : QObject(),
           InterfaceController<proto::UiValues, proto::ModuleState>(net::utils::ChannelType::INTERFACE_TO_AI_CHANNEL, net::utils::ChannelType::AI_TO_INTERFACE_CHANNEL, 20, 20)
    {
        this->fieldState = std::make_shared<InterfaceFieldStateStore>();
        this->fieldSubscriber = std::make_unique<net::WorldSubscriber>([this] (auto state) { field_state_callback(state); });

        this->yellowDataSub = std::make_unique<net::AIYellowDataSubscriber>([&](const AIData& data) {
            this->onAIData(data, Team::YELLOW);
        });
        this->blueDataSub = std::make_unique<net::AIBlueDataSubscriber>([&](const AIData& data) {
            this->onAIData(data, Team::BLUE);
        });

        QObject::connect(&interface_timer, &QTimer::timeout, this, &InterfaceControllerClient::refresh_trigger);
        interface_timer.start(16); // 60 FPS
    }

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
    void InterfaceControllerClient::onAIData(const AIData& data, Team team) {
        this->fieldState->setAIData(data, team);
    }

    void InterfaceControllerClient::stop() {
        this->fieldSubscriber.reset(); // Prevent ugly crashes on exit

        InterfaceController::stop();
    }
}
