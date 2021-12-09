//
// Created by Dawid Kulikowski on 18/08/2021.
//

#ifndef RTT_INTERFACECONTROLLER_H
#define RTT_INTERFACECONTROLLER_H
#include <roboteam_proto/State.pb.h>
#include <roboteam_utils/Print.h>
#include <roboteam_utils/Timer.h>
#include <networking/Pair.hpp>

#include "InterfaceDeclarations.h"
#include "InterfaceSettings.h"
#include "InterfaceStateHandler.h"

namespace rtt::Interface  {
    class InterfaceController {
    private:
        std::shared_ptr<InterfaceDeclarations> decls;
        std::shared_ptr<InterfaceSettings> vals;

        std::shared_ptr<InterfaceStateHandler> declState;
        std::shared_ptr<InterfaceStateHandler> valueState;

        std::unique_ptr<networking::PairReceiver<16971>> conn = std::make_unique<networking::PairReceiver<16971>>();

        roboteam_utils::Timer timerLoop;
        std::thread loopThread;

        void loop();

    public:
        InterfaceController() {
            declState = std::make_shared<InterfaceStateHandler>();
            valueState = std::make_shared<InterfaceStateHandler>();

            decls = std::make_shared<InterfaceDeclarations>(declState);
            vals = std::make_shared<InterfaceSettings>(valueState);
        }

        std::weak_ptr<InterfaceStateHandler> getDeclarationsChangeTracker() {return declState;}
        std::weak_ptr<InterfaceStateHandler> getSettingsChangeTracker() {return valueState;}

        std::weak_ptr<InterfaceDeclarations> getDeclarations() const {return decls;}
        std::weak_ptr<InterfaceSettings> getValues() const {return vals;}

        void handleData(const proto::ModuleState state) {
            // TODO: Handle field state
            if (state.handshakes_size() != 1) {
                return;
            }

            state.PrintDebugString();

            const auto handshake = state.handshakes(0);

            this->decls->handleData(handshake.declarations());
            this->vals->handleData(handshake.values(), decls);
        }

        void sendVals() {
            proto::ModuleState mod;
            proto::Handshake hand;
            hand.mutable_values()->CopyFrom(this->vals.get()->toProto());
            hand.set_module_name("_DEFAULT");
            hand.mutable_declarations()->CopyFrom(this->decls.get()->toProto());
            mod.mutable_handshakes()->Add(std::move(hand));

            this->conn->write(std::move(mod));
        }


        void run();

        void stop() {
            this->timerLoop.stop();
            this->loopThread.join();
        }
    };
}

#endif  // RTT_INTERFACECONTROLLER_H
