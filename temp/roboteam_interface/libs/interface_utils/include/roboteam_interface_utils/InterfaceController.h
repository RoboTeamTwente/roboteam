//
// Created by Dawid Kulikowski on 18/08/2021.
//

#pragma once
#include <proto/State.pb.h>
#include <chrono>
#include <unistd.h>


#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

#include "InterfaceDeclarations.h"
#include "InterfaceSettings.h"

namespace rtt::Interface  {

    template<typename S, typename R>
    class InterfaceController : private net::utils::Publisher, private net::utils::Subscriber {
    public:
        InterfaceController(net::utils::ChannelType publishChannel, net::utils::ChannelType subscribeChannel)
            : net::utils::Publisher(publishChannel),
              net::utils::Subscriber(subscribeChannel, [&](const std::string& message) {
                  this->onReceivedMessage(message);
              }) {
            this->shouldRun = true;

            this->decls = std::make_shared<InterfaceDeclarations>();
            this->vals = std::make_shared<InterfaceSettings>();
        }

        [[nodiscard]] std::weak_ptr<InterfaceDeclarations> getDeclarations() const {
            return this->decls;
        }

        [[nodiscard]] std::weak_ptr<InterfaceSettings> getValues() const {
            return this->vals;
        }

        virtual void loop() = 0;

        virtual void handleData(const R& state) = 0;

        virtual S getDataForRemote() const noexcept = 0;

        void run() {
            std::thread t1(&InterfaceController::loop, this);
            this->loopThread = std::move(t1);
        }

        virtual void stop() {
            this->shouldRun.store(false);
            this->loopThread.join();
        }

    protected:
        std::atomic_bool shouldRun;

        std::shared_ptr<InterfaceDeclarations> decls;
        std::shared_ptr<InterfaceSettings> vals;

        R recv_state;

        void loop_iter() {
            auto data = getDataForRemote();
            this->send(data.SerializeAsString());
        }


    private:
        std::thread loopThread;

        void onReceivedMessage(const std::string& message) {
            this->recv_state.ParseFromString(message);
            this->handleData(this->recv_state);
        }
    };

} // namespace rtt::Interface
