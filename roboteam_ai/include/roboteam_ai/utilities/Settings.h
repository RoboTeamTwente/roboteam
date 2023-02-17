#pragma once

#include <string>

namespace rtt {

/**
 * @brief Class that defines the settings handler for the interface
 */
class Settings {
   public:
    /**
     * @brief Enumerator that tells where RobotHub should send the robot commands
     */
    enum RobotHubMode { UNKNOWN, BASESTATION, SIMULATOR };

    /**
     * @brief Constructor for the Settings class
     */
    Settings();

    /**
     * @brief Initializer for the settings handler
     * @param idOfAI ID of the AI
     */
    void init(int idOfAI);
    /**
     * This function takes directly the values of the settings of Primary AI, and will convert them to settings this AI should have.
     * @param isYellow Indicates whether this is they yellow team
     * @param isLeft Indicates whether we are playing on the left side
     * @param mode Mode of RobotHub
     * @param visionIp IP address where vision is published
     * @param visionPort Port where vision is published
     * @param refereeIp IP address where the referee is published
     * @param refereePort Port where the referee is published
     * @param robotHubSendIp IP address on which RobotHubs should send the robot commands
     * @param robotHubSendPort Port on which RobotHub should send the robot commands
     */
    void handleSettingsFromPrimaryAI(bool isYellow, bool isLeft, RobotHubMode mode, std::string visionIp, int visionPort, std::string refereeIp, int refereePort,
                                     std::string robotHubSendIp, int robotHubSendPort);

    /**
     * @brief Checks whether this AI is the primary AI
     * @return Boolean that tells whethewr this AI is the primary AI
     */
    bool isPrimaryAI() const;

    /**
     * @brief Gets the ID of this AI
     * @return The ID of this AI
     */
    int getId() const;

    /**
     * @brief Sets the ID of this AI
     * @param id The ID of this AI
     */
    void setId(int id);

    /**
     * @brief Checks whether this is the yellow team
     * @return Boolean that tells whether this is the yellow team
     */
    bool isYellow() const;

    /**
     * @brief Sets the given team
     * @param yellow Indicates whether this is the yellow team
     * @return Boolean that tells whether the team has been set succesfully
     */
    bool setYellow(bool yellow);

    /**
     * @brief Checks whether we are playing on the left side
     * @return Boolean that tells whether we are on the left side
     */
    bool isLeft() const;

    /**
     * @brief Sets the side we are playing on
     * @param left Indicates whether we are playing on the left side
     */
    void setLeft(bool left);

    /**
     * @brief Gets the mode that RobotHub is in
     * @return The mode that RobotHub is in
     */
    RobotHubMode getRobotHubMode() const;

    /**
     * @brief Sets the RobotHub mode
     * @param mode The mode RobotHub should be in
     * @return Boolean that tells whether the mode has been set successfully
     */
    bool setRobotHubMode(RobotHubMode mode);

    /**
     * @brief Gets the IP address where vision is published
     * @return The IP address where vision is published
     */
    const std::string &getVisionIp() const;

    /**
     * @brief Sets the IP address where vision is published
     * @param visionIp The IP address where vision is published
     */
    void setVisionIp(const std::string &visionIp);

    /**
     * @brief Gets the port where vision is published
     * @return The port where vision is published
     */
    int getVisionPort() const;

    /**
     * @brief Sets the port where vision is published
     * @param visionPort The port where vision is published
     */
    void setVisionPort(int visionPort);

    /**
     * @brief Gets the IP address where the referee is published
     * @return The IP address where the referee is published
     */
    const std::string &getRefereeIp() const;

    /**
     * @brief Sets the IP address where the referee is published
     * @param refereeIp The IP address where the referee is published
     */
    void setRefereeIp(const std::string &refereeIp);

    /**
     * @brief Gets the port where the referee is published
     * @return The port where the referee is published
     */
    int getRefereePort() const;

    /**
     * @brief Sets the port where the referee is published
     * @param refereePort The port where the refree is pubhlished
     */
    void setRefereePort(int refereePort);

    /**
     * @brief Gets the IP address where RobotHub sends the robot commands
     * @return The IP address where RobotHub sends the robot commands
     */
    const std::string &getRobothubSendIp() const;

    /**
     * @brief Sets the IP address where RobotHub sends the robot commands
     * @param robothubSendIp The IP address where RobotHub sends the robot commands
     */
    void setRobothubSendIp(const std::string &robothubSendIp);

    /**
     * @brief Gets the port where RobotHub sends the robot commands
     * @return The port where RobotHub sends the robot commands
     */
    int getRobothubSendPort() const;

    /**
     * @brief Sets the port where RobotHub sends the robot commands
     * @param robothubSendPort The port where RobotHub sends the robot commands
     */
    void setRobothubSendPort(int robothubSendPort);

    /**
     * @brief Turns the mode in which RobotHub is sending the robot commands, into a string
     * @param mode The mode in which RobotHub is sending the robot commands
     * @return A string with name of the mode in which RobotHub is sending the robot commands
     */
    static std::string robotHubModeToString(RobotHubMode mode);

   private:
    int id = 0; /**< ID of the AI */
    bool yellow = true; /**< Indicates whether this is the yellow team */
    bool left = true; /**< Indicates whether we are playing on the left side */
    RobotHubMode robotHubMode; /**< The mode in which RobotHub is sending robot commands */

    std::string visionIp; /**< IP address where vision is published */
    int visionPort; /**< Port where the vision is published */
    std::string refereeIp; /**< IP address where the referee is published */
    int refereePort; /**< Port where the referee is published */
    std::string robothubSendIp; /**< IP address where RobotHub is sending the robot commands */
    int robothubSendPort; /**< IP address where RobotHub is sending the robot commands */
};

extern Settings SETTINGS;

}  // namespace rtt
