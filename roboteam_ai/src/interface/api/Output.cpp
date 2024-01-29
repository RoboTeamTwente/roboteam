#include "interface/api/Output.h"

namespace rtt::ai::interface {

// these values are default initialized here, but will be updated once mainWidget.cpp constructs the PID widget.
pidVals Output::numTreePID = Constants::standardNumTreePID();
pidVals Output::receivePID = Constants::standardReceivePID();
pidVals Output::interceptPID = Constants::standardInterceptPID();
pidVals Output::keeperPID = Constants::standardKeeperPID();
pidVals Output::keeperInterceptPID = Constants::standardKeeperInterceptPID();

rtt::Vector2 Output::markerPosition = {0, 0};  // initialize on middle of the field
bool Output::useRefereeCommands = false;

std::mutex Output::markerMutex;
std::mutex Output::refMutex;

GameState Output::interfaceGameState("halt_strategy", Constants::RULESET_DEFAULT());

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

void Output::setRuleSetName(std::string name) { Output::interfaceGameState.ruleSet.title = std::move(name); }

void Output::setKeeperId(int id) { Output::interfaceGameState.keeperId = id; }

const GameState &Output::getInterfaceGameState() { return Output::interfaceGameState; }

void Output::setInterfaceGameState(GameState interfaceGameState) {
    // keep the keeper the same
    interfaceGameState.keeperId = Output::interfaceGameState.keeperId;
    Output::interfaceGameState = interfaceGameState;
}

const pidVals &Output::getNumTreePid() { return numTreePID; }

void Output::setNumTreePid(const pidVals &numTreePid) { numTreePID = numTreePid; }

const pidVals &Output::getReceivePid() { return receivePID; }

void Output::setReceivePid(const pidVals &receivePid) { receivePID = receivePid; }

const pidVals &Output::getInterceptPid() { return interceptPID; }

void Output::setInterceptPid(const pidVals &interceptPid) { interceptPID = interceptPid; }

const pidVals &Output::getKeeperPid() { return keeperPID; }

void Output::setKeeperPid(const pidVals &keeperPid) { keeperPID = keeperPid; }

const pidVals &Output::getKeeperInterceptPid() { return keeperInterceptPID; }

void Output::setKeeperInterceptPid(const pidVals &keeperInterceptPid) { keeperInterceptPID = keeperInterceptPid; }

}  // namespace rtt::ai::interface