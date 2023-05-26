#ifndef ROBOTEAM_AI_IO_MANAGER_H
#define ROBOTEAM_AI_IO_MANAGER_H

#include <proto/SimulationConfiguration.pb.h>
#include <utilities/Constants.h>
#include <utilities/GameSettings.h>

#include <RobotCommandsNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <SimulationConfigurationNetworker.hpp>
#include <WorldNetworker.hpp>
#include <iostream>
#include <mutex>
#include <roboteam_utils/Field.hpp>

namespace rtt::world {
class World;
}

namespace rtt::ai {
class Pause;

namespace io {
using namespace rtt::world;

/**
 * @brief Class that defines the IOManager. The IOManager manages the input and output of AI.
 * The input consists of world and the output of simulation configuration, robot commands and settings.
 */
class IOManager {
   private:
    proto::State state; /**< State given by the roboteam_observer */
    uint64_t stateTimeMs; /** Time of the last state update in milliseconds since epoch */
    uint64_t stateWorldLastTimestamp; /** Timestamp of the last world update in the state */

    std::unique_ptr<rtt::net::WorldSubscriber> worldSubscriber; /**< The socket that receives the world information */
    std::unique_ptr<rtt::net::SimulationConfigurationPublisher> simulationConfigurationPublisher; /**< The socket that publishes the configuration for the simulator */

    /**
     * @brief Unpacks the state so it is usable by AI
     * @param state The state that needs to be unpacked
     */
    void handleState(const proto::State& state);

    std::unique_ptr<rtt::net::RobotCommandsBluePublisher> robotCommandsBluePublisher; /**< The socket that publishes the robot commands for the blue team */
    std::unique_ptr<rtt::net::RobotCommandsYellowPublisher> robotCommandsYellowPublisher; /**< The socket that publishes the robot commands for the yellow team */

    /** Only the primary AI publishes settings. The secondary AI subscribes to those settings so they are on the same line */
    std::unique_ptr<rtt::net::SettingsPublisher> settingsPublisher; /**< The socket that publishes the settings from interface */
    std::unique_ptr<rtt::net::SettingsSubscriber> settingsSubscriber; /**< The socket that receives the settings for interface */

    rtt::ai::Pause* pause; /**< Pauses the robots when needed */

    /**
     * @brief Adds the camera angle from world to the robot commands, so the robot can use it for its angle control
     * @param robotCommands the robot commands in which the camera angle needs to be added
     */
    void addCameraAngleToRobotCommands(rtt::RobotCommands& robotCommands);
    /**
     * @brief Publishes the robot commands on the given team's socket.
     * @param robotCommands Commands that need to be published
     * @param isForTeamYellow Indicates whether the robot commands are for the yellow team
     * @return Boolean that tells whether the commands have been sent successfully
     */
    bool publishRobotCommands(const rtt::RobotCommands& robotCommands, bool isForTeamYellow);

   public:
    /**
     * @brief Publishes all robot commands with the robot angles from world
     * @param robotCommands Commands that need to be published
     */
    void publishAllRobotCommands(rtt::RobotCommands& robotCommands);

    /**
     * @brief Publishes the settings on the settingsPublisher channel
     * @param settings The settings that need to be published
     */
    void publishSettings();

    /**
     * @brief Publishes the simulation configuration on the simulationConfiguration channel. Only the primary AI is allowed to send this.
     * @param configuration The simulation configuration that needs to be published
     * @return Boolean that tells whether the simulation configuration has been sent successfully
     */
    bool sendSimulationConfiguration(const proto::SimulationConfiguration& configuration);

    /**
     * @brief Initializes the IOManager
     * @param isPrimaryAI Indicates whether this is the primary AI
     * @return Boolean that tells whether the IOManager is initialized successfully
     */
    bool init(bool isPrimaryAI);

    /**
     * @brief Gets the state published by the referee
     * @return The state
     */
    proto::State getState();

    /**
     * @brief Get the time of the last state update in milliseconds since epoch
     * @return The time 
     */
    uint64_t getStateTimeMs();

    /**
     * @brief Get the age of the last state update in milliseconds
     * @return The age
     */
    uint64_t getStateAgeMs();

    /**
     * @brief Tries to set up a socket on the given channel
     * @param yellowChannel Indicates whether the socket should bind to the yellow channel
     * @return Boolean that tells whether the socket is set up successfully
     */
    bool obtainTeamColorChannel(bool yellowChannel);

    std::mutex stateMutex; /**< Synchronizer for the state */
};

extern IOManager io;

}  // namespace io
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_IO_MANAGER_H