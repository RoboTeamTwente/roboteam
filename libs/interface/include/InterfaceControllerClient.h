//
// Created by Dawid Kulikowski on 06/01/2022.
//
#pragma once

#include <roboteam_interface_utils/InterfaceController.h>
#include <WorldNetworker.hpp>
#include <AIDataNetworker.hpp>
#include <QObject>
#include <QTimer>

#include <utils/Channels.hpp>
#include "InterfaceFieldStateStore.h"

namespace rtt::Interface {
    class InterfaceControllerClient: public QObject, public InterfaceController<proto::UiValues, proto::ModuleState>  {
        Q_OBJECT
    public:
        std::weak_ptr<InterfaceFieldStateStore> getFieldState() const;
        InterfaceControllerClient();
        void stop() override;
    signals:
        void refresh_trigger();

    private:
        QTimer interface_timer;

        std::unique_ptr<net::WorldSubscriber> fieldSubscriber;
        std::unique_ptr<net::AIYellowDataSubscriber> yellowDataSub;
        std::unique_ptr<net::AIBlueDataSubscriber> blueDataSub;

        std::shared_ptr<InterfaceFieldStateStore> fieldState;
        void field_state_callback(const proto::State&);
        void onAIData(const AIData& data, Team team);

        proto::UiValues getDataForRemote(bool) const noexcept override;
        virtual void handleData(const proto::ModuleState& state) override;
        virtual bool hasPriorityData() const noexcept override;
    };
} // namespace rtt::Interface
