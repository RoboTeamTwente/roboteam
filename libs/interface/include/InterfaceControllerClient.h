//
// Created by Dawid Kulikowski on 06/01/2022.
//

#ifndef RTT_INTERFACECONTROLLERCLIENT_H
#define RTT_INTERFACECONTROLLERCLIENT_H


#include <roboteam_interface_utils/InterfaceController.h>
#include <WorldNetworker.hpp>
#include <QObject>
#include <QTimer>

#include "InterfaceFieldStateStore.h"

namespace rtt::Interface {
    class InterfaceControllerClient: public QObject, public InterfaceController<16971, 20, 20, proto::UiValues, proto::ModuleState>  {
        Q_OBJECT
    public:
        std::weak_ptr<InterfaceFieldStateStore> getFieldState() const;
        InterfaceControllerClient(): QObject(), fieldState(std::make_shared<InterfaceFieldStateStore>()), InterfaceController<16971, 20, 20, proto::UiValues, proto::ModuleState>(), field_subscriber(std::make_unique<rtt::net::WorldSubscriber>([this] (auto state) { field_state_callback(state); })) {
            QObject::connect(&interface_timer, &QTimer::timeout, this, &InterfaceControllerClient::refresh_trigger);
            interface_timer.start(16); // 60 FPS
        }
        void stop() override;
    signals:
        void refresh_trigger();

    private:
        QTimer interface_timer;

        std::unique_ptr<rtt::net::WorldSubscriber> field_subscriber;

        std::shared_ptr<InterfaceFieldStateStore> fieldState;
        void field_state_callback(const proto::State&);
        proto::UiValues getDataForRemote(bool) const noexcept override;
        virtual void handleData(const proto::ModuleState& state) override;
        virtual bool hasPriorityData() const noexcept override;
    };
}


#endif  // RTT_INTERFACECONTROLLERCLIENT_H
