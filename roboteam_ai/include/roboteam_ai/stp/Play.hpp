#ifndef RTT_PLAY_HPP
#define RTT_PLAY_HPP

#include <array>

#include "PlayEvaluator.h"
#include "Role.hpp"
#include "computations/PositionComputations.h"
#include "stp/evaluations/BaseEvaluation.h"
#include "utilities/Dealer.h"
#include "utilities/GameState.h"
#include "world/World.hpp"

namespace rtt::ai::stp {
using pos = PositionComputations;
using eval = GlobalEvaluation;

/**
 * @brief Play class that's used in the STP model
 * on update traverses every Role, and updates it.
 */
class Play {
   public:
    std::vector<GlobalEvaluation> keepPlayEvaluation; /**< Invariant vector that contains invariants that need to be true to continue execution of this play */

    std::vector<GlobalEvaluation> startPlayEvaluation; /**< Invariant vector that contains invariants that need to be true to start this play */

    static int waller_count;

    /**
     * @brief Initializes stpInfos struct, distributes roles, sets the previousRobotNum variable and calls onInitialize()
     */
    void initialize() noexcept;

    /**
     * @brief Sets the Play's world pointer to the static world class
     * @param pointer to World
     */
    void setWorld(world::World* world) noexcept;

    /**
     * @brief Updates the field in the play
     * @param field the current field
     */
    void updateField(Field field) noexcept;

    /**
     * @brief Updates (or ticks) all the roles that have robots assigned to them
     */
    virtual void update() noexcept;

    /**
     * @brief Calculates all the info the roles need in order to execute correctly.
     * This is a purely virtual function, so it is implemented in every play.
     */
    virtual void calculateInfoForRoles() noexcept = 0;

    /**
     * @brief Scores the play based on how effective this play would be given the current world
     * @param field The current Field class
     * @return Score of the play (0 - 255)
     */
    virtual uint8_t score(const rtt::Field& field) noexcept = 0;

    /**
     * @brief Virtual default destructor, ensures proper destruction of derived plays
     */
    virtual ~Play() = default;

    /**
     * @brief Default constructor, proper construction
     */
    Play() = default;

    /**
     * @brief Default move-constructor, ensures proper move-construction of Play
     * @param other Play that needs to be moved
     */
    Play(Play&& other) = default;

    /**
     * @brief Check if the preconditions of this play are true
     * @return true if the play is allowed to be started, else false
     */
    [[nodiscard]] bool isValidPlayToStart() const noexcept;

    /**
     * @brief Check if the invariants necessary to keep this play are true
     * @return true if the play is valid to keep, else false
     */
    [[nodiscard]] bool isValidPlayToKeep() noexcept;

    /**
     * @brief Getter for the role -> status mapping
     * @return The internal role -> status mapping, roleStatuses
     */
    [[nodiscard]] std::unordered_map<Role*, Status> const& getRoleStatuses() const;

    /**
     * @brief Gets the current play name
     * @return name of the play
     */
    virtual const char* getName() const = 0;

    std::optional<uint8_t> lastScore; /**< If score was calculated, save here */

    /**
     * @brief Gets the last known score
     * @return score, if no value -> 0
     */
    uint8_t getLastScore() const;

   protected:
    std::array<std::unique_ptr<Role>, rtt::ai::constants::MAX_ROBOT_COUNT> roles; /**< The roles, constructed in ctor of a play */

    std::vector<PlayEvaluator::PlayScoring> scoring; /**< The evaluations with their weight */

    std::unordered_map<Role*, Status> roleStatuses; /**< Map that keeps track of the status of each role. It's a Role*, because that's hashable and a unique identifier */

    std::unordered_map<std::string, StpInfo> stpInfos; /**< The stpInfos. The string is the role_name to be able to update the info in the right role*/

    rtt::world::World* world{}; /**< The world pointer */

    rtt::Field field; /**< The Field */

    /**
     * @brief Decides the input for the robot dealer. The result will be used to distribute the roles
     * @return a mapping between roles and robot flags, used by the robot dealer to assign roles
     */
    virtual Dealer::FlagMap decideRoleFlags() const noexcept = 0;

    /**
     * @brief Optional function to force end plays
     * @return True if play should end this tick
     */
    virtual bool shouldEndPlay() noexcept;

   private:
    /**
     * @brief This function refreshes the RobotViews, BallViews, and Fields for all StpInfo's. This also sets the maxRobotVelocity.
     * This is necessary because the views are stored for a limited time; not refreshing will lead to UB
     */
    void refreshData() noexcept;

    /**
     * @brief Assigns robots to roles
     */
    void distributeRoles() noexcept;

    /**
     * @brief Re-calculates info for roles and reassigns robots.
     * This function is only used when the amount of robots in the field changed compared to the previous tick
     */
    void reassignRobots() noexcept;

    /**
     * @brief Draws the margins for the defence area, ball and cardRobot.
     * This function will draw margins in the interfacce
     */
    void DrawMargins() noexcept;

    size_t previousRobotNum{}; /**< The previous amount of robots. This is used to check if we need to re-deal (if a robot disappears for example) */

    int previousKeeperId = -1;  /**< The previous keeperId. This is used to check if we need to re-deal (if keeper id was changed from UI or GameController) */
    int previousMaxRobots = -1; /**< The previous maxRobots. This is used to check if we need to re-deal */
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAY_HPP
