#include "interface/api/Output.h"

namespace rtt::ai::interface {

bool Output::useRefereeCommands = false;

std::mutex Output::refMutex;
std::mutex Output::interfaceGameStateMutex;

GameState Output::interfaceGameState(RefCommand::HALT, RuleSet::RULESET_HALT());

bool Output::usesRefereeCommands() {
    std::lock_guard<std::mutex> lock(refMutex);
    return useRefereeCommands;
}

void Output::setUseRefereeCommands(bool useRefereeCommands) {
    std::lock_guard<std::mutex> lock(refMutex);
    Output::useRefereeCommands = useRefereeCommands;
}

void Output::setRuleSetName(std::string name) {
    std::lock_guard<std::mutex> lock(interfaceGameStateMutex);
    Output::interfaceGameState.ruleSet.toString() = std::move(name);
}

void Output::setKeeperId(int id) {
    std::lock_guard<std::mutex> lock(interfaceGameStateMutex);
    Output::interfaceGameState.keeperId = id;
}

const GameState &Output::getInterfaceGameState() {
    std::lock_guard<std::mutex> lock(interfaceGameStateMutex);
    return Output::interfaceGameState;
}

void Output::setInterfaceGameState(GameState interfaceGameState) {
    std::lock_guard<std::mutex> lock(interfaceGameStateMutex);
    interfaceGameState.keeperId = Output::interfaceGameState.keeperId;
    Output::interfaceGameState = interfaceGameState;
}
}  // namespace rtt::ai::interface