//
// Created by Dawid Kulikowski on 18/08/2021.
//

#ifndef RTT_INTERFACECONTROLLER_H
#define RTT_INTERFACECONTROLLER_H
#include <proto/State.pb.h>
#include <chrono>
#include <utils/Pair.hpp>

#include "InterfaceDeclarations.h"
#include "InterfaceSettings.h"
// Port: 16971
namespace rtt::Interface  {

    namespace networking = rtt::net::utils;

    template<size_t port, uint8_t throttle, uint8_t max_time_between_remote_updates, typename S, typename R>
    class InterfaceController {
    private:
        std::unique_ptr<networking::PairReceiver<port>> conn;

        std::thread loopThread;

        std::atomic_bool should_run = true;
        zmqpp::poller poller;

        void loop() {
            using namespace std::chrono_literals;
            this->last_state_update = std::chrono::high_resolution_clock::now();
            poller.add(conn->socket);

            while (this->should_run) {

                bool has_data = poller.poll(max_time_between_remote_updates);

                auto time_now = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now-this->last_state_update).count();

                if (hasPriorityData() || duration >= max_time_between_remote_updates) {
                    this->conn->write(getDataForRemote(duration >= max_time_between_remote_updates), true);
                }

                if (!has_data) continue;

                std::string msg;
                if (!this->conn->socket.receive(msg)) {
                    std::cout << "[INTERFACE] Invalid message from peer!" << std::endl;
                    continue;
                }

                if (duration >= throttle) {
                    recv_state.ParseFromString(msg); // Prevent needless protobuf parsing and memory allocations
                    this->handleData(recv_state); // Pass by reference, make a copy when required

                    this->last_state_update = std::chrono::high_resolution_clock::now();
                }
            }

            poller.remove(conn->socket);
            conn->socket.close();
        }
    protected:
        std::shared_ptr<InterfaceDeclarations> decls;
        std::shared_ptr<InterfaceSettings> vals;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_state_update;
        R recv_state;


    public:
        InterfaceController(): decls(std::make_shared<InterfaceDeclarations>()), vals(std::make_shared<InterfaceSettings>()), conn(std::make_unique<networking::PairReceiver<port>>()) {
            this->conn->socket.set(zmqpp::socket_option::linger, 0);
        } //, fieldState(std::make_shared<InterfaceFieldStateStore>())

        [[nodiscard]] std::weak_ptr<InterfaceDeclarations> getDeclarations() const {return decls;}
        [[nodiscard]] std::weak_ptr<InterfaceSettings> getValues() const {return vals;}

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
            this->should_run.store(false);
            this->loopThread.join();
        }
    };
}

#endif  // RTT_INTERFACECONTROLLER_H
