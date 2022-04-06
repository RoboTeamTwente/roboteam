//
// Created by Dawid Kulikowski on 06/01/2022.
//

#ifndef RTT_INTERFACECONTROLLERCLIENT_H
#define RTT_INTERFACECONTROLLERCLIENT_H


#include <roboteam_interface_utils/InterfaceController.h>

#include <utils/Channels.hpp>
#include "InterfaceFieldStateStore.h"

namespace rtt::Interface {
    class InterfaceControllerClient: public InterfaceController<proto::UiValues, proto::ModuleState> {
    public:
        std::weak_ptr<InterfaceFieldStateStore> getFieldState() const;
        InterfaceControllerClient(): InterfaceController<proto::UiValues, proto::ModuleState>(rtt::net::utils::ChannelType::INTERFACE_TO_AI_CHANNEL, rtt::net::utils::ChannelType::AI_TO_INTERFACE_CHANNEL, 20, 20), fieldState(std::make_shared<InterfaceFieldStateStore>()) {}
    private:
        std::shared_ptr<InterfaceFieldStateStore> fieldState;
        proto::UiValues getDataForRemote(bool) const noexcept override;
        virtual void handleData(const proto::ModuleState& state) override;
        virtual bool hasPriorityData() const noexcept override;
    };
}


#endif  // RTT_INTERFACECONTROLLERCLIENT_H
