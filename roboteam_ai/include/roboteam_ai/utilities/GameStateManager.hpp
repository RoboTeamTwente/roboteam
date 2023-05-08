#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include <proto/messages_robocup_ssl_referee.pb.h>

#include "RefGameState.h"
#include "StrategyManager.h"
#include "world/views/WorldDataView.hpp"

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
    static void setGameStateFromReferee(proto::SSL_Referee refMsg, std::optional<world::view::WorldDataView> world);

    static void setGameStateFromInterface(std::string_view playName, std::optional<std::string_view> ruleset, std::optional<int> keeperId);

    /**
     * @brief Force a new game state. This overrules the referee
     * @param cmd The game state we want
     * @param ball The current ball data
     */
    static void forceNewGameState(RefCommand cmd, std::optional<rtt::world::view::BallView> ball);

    /**
     * @brief Getter for the referee data
     * @return The referee data
     */
    static proto::SSL_Referee getRefereeData();

    /**
     * @brief Getter for the current game state
     * @return The current game state
     */
    static GameState getCurrentGameState();

    /**
     * @brief Getter for a designated position from the referee. For example: ball placement position
     * @return Designated position from the referee
     */
    static Vector2 getRefereeDesignatedPosition();

   private:
    static const std::unordered_map<std::string_view, GameState> playNameGameStateMapping;
    static const std::unordered_map<RefCommand, RefGameState> refCommandToGameStateMapping;

    static RefGameState findNextRefGameState(const proto::SSL_Referee_Stage& stage, RefCommand newRefCmd, std::optional<world::view::BallView> ballView);

    static RefGameState currentRefGameState;
    static GameState currentInterfaceGameState;
    static proto::SSL_Referee refMsg; /**< Data from the referee */

    static std::mutex lock;
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_GAMESTATEMANAGER_HPP
