//
// Created by john on 3/9/20.
//

#include "stp/PlayDecider.hpp"

namespace rtt::ai::stp {

    bool PlayDecider::interfacePlayChanged = false;

    Play *PlayDecider::decideBestPlay(world::World *pWorld, std::vector<Play *> plays) noexcept {
        if (interfacePlay) {
            return interfacePlay;
        }
        return *std::max_element(plays.begin(), plays.end(), [&](auto &largest, auto &play) { return largest->score(pWorld) < play->score(pWorld); });
    }

    // This is only used by the interface to force new plays
    void PlayDecider::lockInterfacePlay(Play *play) {
        PlayDecider::interfacePlayChanged = true;
        interfacePlay = play;
    }
}  // namespace rtt::ai::stp