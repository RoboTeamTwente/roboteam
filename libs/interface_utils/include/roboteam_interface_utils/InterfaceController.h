//
// Created by Dawid Kulikowski on 18/08/2021.
//

#pragma once
#include <proto/State.pb.h>
#include <chrono>

#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

#include <roboteam_interface_utils/InterfaceController.h>
#include <roboteam_interface_utils/InterfaceSettings.h>

namespace rtt::Interface  {

template<typename S, typename R>
class InterfaceController {
public:
    InterfaceController(net::utils::ChannelType publishChannel, net::utils::ChannelType subscribeChannel, uint8_t receiveThrottle, uint8_t maxTimeBetweenRemoteUpdates) {
        this->receiveThrottle = receiveThrottle;
        this->maxTimeBetweenRemoteUpdates = maxTimeBetweenRemoteUpdates;
        this->shouldRun = true;

        this->decls = std::make_shared<InterfaceDeclarations>();
        this->vals = std::make_shared<InterfaceSettings>();

        this->publisher = std::make_unique<net::utils::Publisher>(publishChannel);
        this->subscriber = std::make_unique<net::utils::Subscriber>(subscribeChannel, [&](const std::string& message) {
            this->onReceivedMessage(message);
        });
    }

    [[nodiscard]] std::weak_ptr<InterfaceDeclarations> getDeclarations() const {
        return this->decls;
    }
    [[nodiscard]] std::weak_ptr<InterfaceSettings> getValues() const {
        return this->vals;
    }

    virtual void handleData(const R& state) = 0;

    virtual S getDataForRemote(bool) const noexcept = 0;

    virtual bool hasPriorityData() const noexcept {
        return false;
    }

    void run() {
        std::thread t1(&InterfaceController::loop, this);
        this->loopThread = std::move(t1);
    }

    void stop() {
        this->shouldRun.store(false);
        this->loopThread.join();
    }

protected:
    std::shared_ptr<InterfaceDeclarations> decls;
    std::shared_ptr<InterfaceSettings> vals;
    // TODO: Use steady_clock instead
    std::chrono::time_point<std::chrono::high_resolution_clock> lastSentData;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastReceivedData;
    R recv_state;


private:
    std::unique_ptr<net::utils::Publisher> publisher;
    std::unique_ptr<net::utils::Subscriber> subscriber;

    std::atomic_bool shouldRun;
    std::thread loopThread;

    zmqpp::poller poller;

    uint8_t receiveThrottle;
    uint8_t maxTimeBetweenRemoteUpdates;

    void loop() {
        while (this->shouldRun) {
            auto time_now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - this->lastSentData).count();

            if (this->hasPriorityData() || duration >= this->maxTimeBetweenRemoteUpdates) {
                auto data = getDataForRemote(duration >= this->maxTimeBetweenRemoteUpdates);
                this->publisher->send(data.SerializeAsString());
                this->lastSentData = time_now;
            }
        }

        this->publisher.reset();
        this->subscriber.reset();
    }

    void onReceivedMessage(const std::string& message) {
        auto time_now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - this->lastReceivedData).count();

        if (duration >= this->receiveThrottle) {
            this->recv_state.ParseFromString(message);
            this->handleData(this->recv_state);

            this->lastReceivedData = time_now;
        }
    }
};

} // namespace rtt::Interface
