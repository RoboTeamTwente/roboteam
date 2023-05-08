#ifndef ROBOTEAM_AI_GAMESTATEMANAGER_HPP
#define ROBOTEAM_AI_GAMESTATEMANAGER_HPP

#include <proto/messages_robocup_ssl_referee.pb.h>

#include "RefGameState.h"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai {
/**
 * @brief Class that describes the game state manager. The game state manager will manage the game state and its rules
 */
class GameStateManager {
   public:
    /**
     * @brief Sets the game state based on the referee data (i.e. when playing with a referee)
     * @param refMsg The data we get from the referee
     * @param world world view
     */
    static void setGameStateFromReferee(proto::SSL_Referee refMsg, std::optional<world::view::WorldDataView> world);

    /**
     * @brief Sets the game state based on message from the interface (i.e. when playing WITHOUT a referee)
     * @param playName The name of the play we want to play received from the interface
     * @param ruleset The ruleset we want to use received from the interface
     * @param keeperId The id of the keeper we want to use received from the interface
     */
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
     * @brief Getter for the current game state (either the referee or the interface one based on useReferee flag)
     * @return The current game state
     */
    static GameState getCurrentGameState();

    /**
     * @brief Getter for a designated position from the referee. For example: ball placement position
     * @return Designated position from the referee
     */
    static Vector2 getRefereeDesignatedPosition();

   private:
    static const std::unordered_map<std::string_view, GameState> playNameGameStateMapping; ///* Mapping of play name to a game state used when setting gamestate from the interface  */
    static const std::unordered_map<RefCommand, RefGameState> refCommandToGameStateMapping; ///* Mapping of ref commands to a game state */

    /**
     * @brief Finds the next game state based on multiple parameters (current ref command, new ref command, ball position etc.)
     *        This is important because some commands have a follow up command or command interpretation could depend on the SSL_Referee_Stage or ball position
     * @return The next game state
     */
    static RefGameState findNextRefGameState(const proto::SSL_Referee_Stage& stage, RefCommand newRefCmd, std::optional<world::view::BallView> ballView);

    static RefGameState currentRefGameState;
    static GameState currentInterfaceGameState;
    static proto::SSL_Referee refMsg; /** Data from the referee */

    static std::mutex lock;
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_GAMESTATEMANAGER_HPP
