#include "interface/api/Output.h"

namespace rtt::ai::interface {

rtt::Vector2 Output::markerPosition = {0, 0};  // initialize on middle of the field
bool Output::useRefereeCommands = false;

std::mutex Output::markerMutex;
std::mutex Output::refMutex;

GameState Output::interfaceGameState(RefCommand::HALT, Constants::RULESET_DEFAULT());

const Vector2 &Output::getInterfaceMarkerPosition() {
    std::lock_guard<std::mutex> lock(markerMutex);
    return markerPosition;
}

void Output::setMarkerPosition(const Vector2 &ballPlacementTarget) {
    std::lock_guard<std::mutex> lock(markerMutex);
    Output::markerPosition = ballPlacementTarget;
}

bool Output::usesRefereeCommands() {
    std::lock_guard<std::mutex> lock(refMutex);
    return useRefereeCommands;
}

void Output::setUseRefereeCommands(bool useRefereeCommands) {
    std::lock_guard<std::mutex> lock(refMutex);
    Output::useRefereeCommands = useRefereeCommands;
}

void Output::setRuleSetName(std::string name) { Output::interfaceGameState.ruleSet.toString() = std::move(name); }

void Output::setKeeperId(int id) { Output::interfaceGameState.keeperId = id; }

const GameState &Output::getInterfaceGameState() { return Output::interfaceGameState; }

void Output::setInterfaceGameState(GameState interfaceGameState) {
    // keep the keeper the same
    interfaceGameState.keeperId = Output::interfaceGameState.keeperId;
    Output::interfaceGameState = interfaceGameState;
}
}  // namespace rtt::ai::interface