//
// Created by tijmen on 13-01-22.
//

#include <simulation/SimulatorManager.hpp>

int main () {

    using namespace rtt::robothub::simulation;

    // GrSim does not allow a bidirectional udp connection, so it uses different ports for the feedback
    SimulatorNetworkConfiguration config = { .blueFeedbackPort = 30011,
                                             .yellowFeedbackPort = 30012,
                                             .configurationFeedbackPort = 30013 };

    std::unique_ptr<SimulatorManager> manager;

    try {
        manager = std::make_unique<SimulatorManager>(config);
    } catch (FailedToBindPortException e) {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }

    ConfigurationCommand configurationCommand;

    configurationCommand.addRobotLocation(0, true, 1.5, 0, 0, 0, 0, 3.14, true, false);
    configurationCommand.addRobotLocation(1, true, 1.5, -1, 0, 0, 0, 3.14, true, false);
    configurationCommand.addRobotLocation(2, true, 0.5, 0, 0, 0, 0, 3.14, true, false);
    configurationCommand.addRobotLocation(3, true, 1.5, 1, 0, 0, 0, 3.14, true, false);

    configurationCommand.addRobotLocation(0, false, -1.5, 0, 0, 0, 0, 0, true, false);
    configurationCommand.addRobotLocation(1, false, -1.5, -1, 0, 0, 0, 0, true, false);
    configurationCommand.addRobotLocation(2, false, -0.5, 0, 0, 0, 0, 0, true, false);
    configurationCommand.addRobotLocation(3, false, -1.5, 1, 0, 0, 0, 0, true, false);

    configurationCommand.setBallLocation(0, 0, 0, 0, 0, 0, false, false, false);

    for(int i = 4; i < 11; i++) {
        configurationCommand.addRobotLocation(i, true, 0, 0, 0, 0, 0, 0, false, false);
        configurationCommand.addRobotLocation(i, false, 0, 0, 0, 0, 0, 0, false, false);
    }


    manager->sendConfigurationCommand(configurationCommand);

}
