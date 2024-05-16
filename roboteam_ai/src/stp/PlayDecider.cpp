#include "stp/PlayDecider.hpp"

#include <roboteam_utils/Print.h>

#include "utilities/RuntimeConfig.h"

namespace rtt::ai::stp {

Play* PlayDecider::decideBestPlay(const rtt::world::World* world, const std::vector<std::unique_ptr<ai::stp::Play>>& plays) noexcept {
    std::vector<Play*> validPlays;
    // Only add plays that are valid
    for (auto& each : plays) {
        if (RuntimeConfig::ignoreInvariants || each->isValidPlayToStart()) {
            validPlays.push_back(each.get());
        }
    }
    std::vector<std::pair<Play*, uint8_t>> playsWithScores;
    playsWithScores.reserve(validPlays.size());

    auto field = world->getField().value();
    for (const auto& play : validPlays) {
        playsWithScores.emplace_back(play, play->score(field));
    }

    if (playLock.isSet) [[unlikely]] {
        std::scoped_lock lock(playLock.lock);
        return getPlayForName(playLock.playName.value(), plays);
    }

    // If there are no valid plays, default to defend shot
    if (playsWithScores.empty()) {
        RTT_WARNING("No valid plays found!");
        return getPlayForName("Defend Shot", plays);
    }

    return std::max_element(playsWithScores.begin(), playsWithScores.end(), [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; })->first;
}

bool PlayDecider::didLockPlay() noexcept { return playLock.didChange.exchange(false); }

void PlayDecider::lockPlay(const std::optional<std::string> playName) {
    std::scoped_lock lock(playLock.lock);
    playLock.playName = playName;

    playLock.isSet = playName.has_value();
    playLock.didChange = true;
}

void PlayDecider::unlockPlay() {
    std::scoped_lock lock(playLock.lock);
    playLock.isSet = false;
    playLock.didChange = true;
}

Play* PlayDecider::getPlayForName(std::string name, const std::vector<std::unique_ptr<ai::stp::Play>>& plays) {
    auto found = std::find_if(plays.begin(), plays.end(), [&](auto& play) { return play->getName() == name; });
    if (found == plays.end()) {
        RTT_ERROR("Could not find play by name");
        return nullptr;
    }
    return found->get();
}

}  // namespace rtt::ai::stp