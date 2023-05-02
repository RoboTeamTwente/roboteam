#pragma once

#include <atomic>
#include <string>

namespace rtt {

/**
 * Game settings that must be shared between all players and the referee (e.g. who is playing on which side, which team is yellow, etc.)
 * These settings are set by the primary AI, and are read-only for secondary AIs.
 * Importantly referee can also change these settings, thus this class must be thread-safe, since referee is handled in a separate (IO) thread.
 *
 * The AI has to be able to handle any changes to these settings WITHOUT restart. Thus settings such as visions_port that require a restart of
 * the AI, are not included here.
 */
class Settings {
   public:
    static constexpr int PRIMARY_AI_ID = 0;

    Settings() = delete;

    /**
     * @brief Enumerator that tells where RobotHub should send the robot commands
     */
    enum RobotHubMode { UNKNOWN, BASESTATION, SIMULATOR };

    /**
     * This function takes directly the values of the settings of Primary AI, and will convert them to settings this AI should have.
     * @param isYellow Indicates whether this is they yellow team
     * @param isLeft Indicates whether we are playing on the left side
     * @param mode Mode of RobotHub
     */
    static void handleSettingsFromPrimaryAI(bool isYellow, bool isLeft, RobotHubMode mode);

    /**
     * @brief Checks whether this AI is the primary AI
     * @return Boolean that tells whether this AI is the primary AI
     */
    static bool isPrimaryAI();

    /**
     * @brief Gets the ID of this AI
     * @return The ID of this AI
     */
    static int getId();

    /**
     * @brief Sets the ID of this AI
     * @param id The ID of this AI
     */
    static void setId(int id);

    /**
     * @brief Checks whether this is the yellow team
     * @return Boolean that tells whether this is the yellow team
     */
    static bool isYellow();

    /**
     * @brief Sets the given team
     * @param yellow Indicates whether this is the yellow team
     * @return Boolean that tells whether the team has been set succesfully
     */
    static bool setYellow(bool yellow);

    /**
     * @brief Checks whether we are playing on the left side
     * @return Boolean that tells whether we are on the left side
     */
    static bool isLeft();

    /**
     * @brief Sets the side we are playing on
     * @param left Indicates whether we are playing on the left side
     */
    static void setLeft(bool left);

    /**
     * @brief Gets the mode that RobotHub is in
     * @return The mode that RobotHub is in
     */
    static  RobotHubMode getRobotHubMode();

    /**
     * @brief Sets the RobotHub mode
     * @param mode The mode RobotHub should be in
     * @return Boolean that tells whether the mode has been set successfully
     */
    static bool setRobotHubMode(RobotHubMode mode);

    /**
     * @brief Turns the mode in which RobotHub is sending the robot commands, into a string
     * @param mode The mode in which RobotHub is sending the robot commands
     * @return A string with name of the mode in which RobotHub is sending the robot commands
     */
    static std::string_view robotHubModeToString(RobotHubMode mode);

   private:
    static std::atomic<int> id; /**< ID of the AI */
    static std::atomic<bool> yellow; /**< Indicates whether this is the yellow team */
    static std::atomic<bool> left; /**< Indicates whether we are playing on the left side */
    static std::atomic<RobotHubMode> robotHubMode; /**< The mode in which RobotHub is sending robot commands */
};

}  // namespace rtt
