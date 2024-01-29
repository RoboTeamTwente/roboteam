#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include <proto/messages_robocup_ssl_referee.pb.h>

#include "RefGameState.h"
#include "StrategyManager.h"

namespace rtt::ai {
/**
 * @brief Class that describes the game state manager. The game state manager will manage the game state and its rules
 */
class GameStateManager {
   public:
    /**
     * @brief Setter for the referee data
     * @param refMsg The data we get from the referee
     * @param data The current world
     */
    static void setRefereeData(proto::Referee refMsg, const rtt::world::World* data);
    /**
     * @brief Getter for the referee data
     * @return The referee data
     */
    static proto::Referee getRefereeData();
    /**
     * @brief Converts the referee command to a RefCommand
     * @param command The command received from the referee
     * @param isYellow Whether our team is the yellow team
     * @return RefCommand The corresponding RefCommand for the received referee command
     */
    static RefCommand getCommandFromRefMsg(proto::Referee_Command command, bool isYellow);
    /**
     * @brief Getter for the current game state
     * @return The current game state
     */
    static GameState getCurrentGameState();
    /**
     * @brief Force a new game state. This overrules the referee
     * @param cmd The game state we want
     */
    static void forceNewGameState(RefCommand cmd);
    /**
     * @brief Getter for a designated position from the referee. For example: ball placement position
     * @return Designated position from the referee
     */
    static Vector2 getRefereeDesignatedPosition();
    /**
     * @brief Updates the interface according to the current game state
     * @param name Name of the current game state
     */
    static void updateInterfaceGameState(const char* name);

   private:
    static proto::Referee refMsg;           /**< Data from the referee */
    static StrategyManager strategymanager; /**< Manager that updates the play according to the game state */
    static std::mutex refMsgLock;           /**< Synchronizer for referee data */
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_GAMESTATEMANAGER_HPP