//
// Created by Dawid Kulikowski on 18/08/2021.
//

#include "InterfaceController.h"
#include <thread>
#include "roboteam_utils/Timer.h"

namespace rtt::Interface {

    void InterfaceController::run() {
        std::thread t1(&InterfaceController::loop, this);
        this->loopThread = std::move(t1);
    }

    void InterfaceController::loop() {
        roboteam_utils::Timer tim;
        tim.loop([&]() {
            if (this->conn->is_ok()) {
                this->sendVals();
                auto read = this->conn->read_next<proto::ModuleState>();
                if (read.is_ok()) {
                    this->handleData(std::move(read.value()));
                }
            }
        }, 60);

        this->timerLoop = std::move(tim);
    }

}
