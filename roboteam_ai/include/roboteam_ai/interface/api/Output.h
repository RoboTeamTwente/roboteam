#ifndef ROBOTEAM_AI_OUTPUT_H
#define ROBOTEAM_AI_OUTPUT_H

#include <mutex>

#include "roboteam_utils/Vector2.h"
#include "utilities/GameState.h"

namespace rtt::ai::interface {

class Output {
   private:
    static std::mutex markerMutex;
    static std::mutex refMutex;

    static rtt::Vector2 markerPosition;
    static bool useRefereeCommands;

    static GameState interfaceGameState;

   public:
    static void sendHaltCommand();

    static void setInterfaceGameState(GameState interfaceGameState);
    static const GameState &getInterfaceGameState();

    static bool usesRefereeCommands();
    static void setUseRefereeCommands(bool useRefereeCommands);
    static const rtt::Vector2 &getInterfaceMarkerPosition();
    static void setMarkerPosition(const rtt::Vector2 &ballPlacementTarget);

    static void setRuleSetName(std::string name);
    static void setKeeperId(int id);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_OUTPUT_H
