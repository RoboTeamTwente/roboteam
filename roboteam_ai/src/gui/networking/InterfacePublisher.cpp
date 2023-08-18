//
// Created by Martin Miksik on 24/05/2023.
//

#include "gui/networking/InterfacePublisher.h"

#include "STPManager.h"
#include "gui/Out.h"
#include "stp/Play.hpp"
#include "utilities/GameSettings.h"
#include "utilities/IOManager.h"
#include "utilities/RuntimeConfig.h"

namespace rtt::ai::gui::net {

InterfacePublisher::InterfacePublisher(ix::WebSocketServer& _wss) : wss(_wss) {}

InterfacePublisher& InterfacePublisher::publishStpStatus(stp::Play* selectedPlay, InterfacePublisher::PlayView plays, int currentTick, double tickDuration, double averageTickDuration) {
    auto envelope = proto::MsgToInterface();
    const auto stpStatus = envelope.mutable_stp_status();
    stpStatus->set_current_tick(currentTick);
    stpStatus->set_tick_duration(tickDuration);
    stpStatus->set_average_tick_duration(averageTickDuration);

    stpStatus->set_score(selectedPlay->lastScore.value_or(-1));

    auto currentPlay = stpStatus->mutable_current_play();
    currentPlay->set_play_name(selectedPlay->getName());

    auto gameState = GameStateManager::getCurrentGameState();
    currentPlay->set_ruleset_name(gameState.getRuleSet().title);
    currentPlay->set_keeper_id(gameState.keeperId);

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
    rtt::ai::gui::Out::consumeVisualizations([&](const proto::MsgToInterface::VisualizationBuffer& visuals) {
        auto envelope = proto::MsgToInterface();
        envelope.mutable_visualizations()->CopyFrom(visuals);
        publishProtoMessage(envelope);
    });

    return *this;
}
InterfacePublisher& InterfacePublisher::publishAIStatus() {
    auto envelope = proto::MsgToInterface();
    const auto aiState = envelope.mutable_ai_state();
    for (auto& play : rtt::STPManager::plays) {  // TODO: How is(was) this thread safe?
        aiState->add_plays(play->getName());
    }

    for (const auto& ruleSet : Constants::ruleSets()) {
        aiState->add_rule_sets(ruleSet.title);
    }

    aiState->set_is_paused(Pause::isPaused());

    const auto game_settings = aiState->mutable_game_settings();
    game_settings->set_robot_hub_mode(robotHubModeToProto(GameSettings::getRobotHubMode()));
    game_settings->set_is_left(GameSettings::isLeft());
    game_settings->set_is_yellow(GameSettings::isYellow());

    const auto ai_settings = aiState->mutable_runtime_config();
    ai_settings->set_use_referee(RuntimeConfig::useReferee);
    ai_settings->set_ignore_invariants(RuntimeConfig::ignoreInvariants);

    publishProtoMessage(envelope);
    return *this;
}

}  // namespace rtt::ai::io