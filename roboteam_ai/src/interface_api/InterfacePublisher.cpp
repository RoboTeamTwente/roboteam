//
// Created by Martin Miksik on 24/05/2023.
//

#include "interface_api/InterfacePublisher.h"

#include "STPManager.h"
#include "interface_api/Out.h"
#include "stp/Play.hpp"
#include "utilities/IOManager.h"

namespace rtt::ai::io {

InterfacePublisher::InterfacePublisher(ix::WebSocketServer& _wss) : wss(_wss) {}

InterfacePublisher& InterfacePublisher::publishStpStatus(stp::Play* selectedPlay, InterfacePublisher::PlayView plays, int currentTick) {
    auto envelope = proto::MsgToInterface();
    const auto stpStatus = envelope.mutable_stp_status();
    stpStatus->set_current_tick(currentTick);
    stpStatus->mutable_selected_play()->set_play_name(selectedPlay->getName());
    stpStatus->mutable_selected_play()->set_play_score(selectedPlay->lastScore.value_or(-1));

    for (const auto& play : plays) {
        auto scoredPlay = stpStatus->add_scored_plays();
        scoredPlay->set_play_name(play->getName());
        scoredPlay->set_play_score(play->getLastScore());
    }

    auto robotsMap = stpStatus->mutable_robots();
    for (auto& [role, status] : selectedPlay->getRoleStatuses()) {
        int robotId = role->getCurrentRobot()->get()->getId();
        (*robotsMap)[robotId] = proto::STPStatus::STPRobot();
        auto& robotMsg = robotsMap->at(robotId);
        robotMsg.set_id(role->getCurrentRobot().value()->getId());

        // Role status
        robotMsg.mutable_role()->set_name(role->getName());
        robotMsg.mutable_role()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(status)});

        // Tactic status
        const auto tactic = role->getCurrentTactic();
        if (!tactic) {
            continue;
        }
        robotMsg.mutable_tactic()->set_name(tactic->getName());
        robotMsg.mutable_tactic()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(tactic->getStatus())});

        // Skill status
        const auto skill = tactic->getCurrentSkill();
        if (!skill) {
            continue;
        }
        robotMsg.mutable_skill()->set_name(skill->getName());
        robotMsg.mutable_skill()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(skill->getStatus())});
    }

    publishProtoMessage(envelope);
    return *this;
}

InterfacePublisher& InterfacePublisher::publishWorld() {
    auto envelope = proto::MsgToInterface();
    auto state = io::io.getState();
    envelope.mutable_state()->CopyFrom(state);
    publishProtoMessage(envelope);

    return *this;
}

InterfacePublisher& InterfacePublisher::publishVisuals() {
    rtt::ai::new_interface::Out::consumeVisualizations([&](const proto::MsgToInterface::VisualizationBuffer& visuals) {
        auto envelope = proto::MsgToInterface();
        envelope.mutable_visualizations()->CopyFrom(visuals);
        publishProtoMessage(envelope);
    });

    return *this;
}

}  // namespace rtt::ai::io