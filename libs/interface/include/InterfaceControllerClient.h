//
// Created by Dawid Kulikowski on 06/01/2022.
//
#pragma once

#include <roboteam_interface_utils/InterfaceController.h>
#include <utils/Subscriber.hpp>
#include <AIDataNetworker.hpp>
#include <QObject>
#include <QTimer>
#include <atomic>

#include <utils/Channels.hpp>
#include "InterfaceFieldStateStore.h"
#include <roboteam_interface_utils/MessageCache.h>

namespace rtt::Interface {
    class InterfaceControllerClient: public QObject, public InterfaceController<proto::UiValues, proto::ModuleState>  {
        Q_OBJECT
    public:
        std::weak_ptr<MessageCache<proto::State>> getFieldState() const;
        InterfaceControllerClient(): QObject(), fieldState(std::make_shared<MessageCache<proto::State>>()), InterfaceController<proto::UiValues, proto::ModuleState>(rtt::net::utils::ChannelType::INTERFACE_TO_AI_CHANNEL, rtt::net::utils::ChannelType::AI_TO_INTERFACE_CHANNEL),
            field_subscriber(std::make_unique<rtt::net::utils::Subscriber>(rtt::net::utils::ChannelType::WORLD_CHANNEL, [this] (auto state) { field_state_callback(state); })) {

            this->aiState = {{Team::BLUE, std::make_shared<MessageCache<AIData>>()}, {Team::YELLOW, std::make_shared<MessageCache<AIData>>()}};

            this->yellowDataSub = std::make_unique<rtt::net::utils::Subscriber>(rtt::net::utils::ChannelType::AI_YELLOW_CHANNEL, [this](auto data) {
                this->aiState.at(Team::YELLOW)->setMessage(data);
            });

            this->blueDataSub = std::make_unique<rtt::net::utils::Subscriber>(rtt::net::utils::ChannelType::AI_BLUE_CHANNEL, [this](auto data) {
                this->aiState.at(Team::BLUE)->setMessage(data);
            });


            QObject::connect(&interface_timer, &QTimer::timeout, this, &InterfaceControllerClient::refresh_trigger);
            interface_timer.start(16); // 60 FPS
        }

        void stop() override;

        void loop() override;

        void markForUpdate();
    signals:
        void refresh_trigger();

    private:
        constexpr static int UPDATE_ATTEMPT_MS = 200; // 1/5 of a second
        constexpr static int MAX_CYCLES_WITHOUT_UPDATE = 5; // 1s

        std::atomic_bool updateMarker;
        int updateCounter = 0;
        QTimer interface_timer;

        std::unique_ptr<rtt::net::utils::Subscriber> field_subscriber;
        std::unique_ptr<rtt::net::utils::Subscriber> yellowDataSub;
        std::unique_ptr<rtt::net::utils::Subscriber> blueDataSub;

        std::shared_ptr<MessageCache<proto::State>> fieldState;

        std::map<Team, std::shared_ptr<MessageCache<AIData>>> aiState;

        void field_state_callback(const std::string&);

        proto::UiValues getDataForRemote() const noexcept override;
        void handleData(const proto::ModuleState& state) override;
    };
} // namespace rtt::Interface
