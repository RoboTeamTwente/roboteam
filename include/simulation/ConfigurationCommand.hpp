#pragma once

#include <ssl_simulation_control.pb.h>

#include <simulation/RobotProperties.hpp>

namespace rtt::robothub::simulation {
/*  This class contains command information to configure and setup the simulator.
    It immediately stores the given information into a packet that can be sent to the simulator.
    Things to configure are robot positions, velocities, ball positions, physical properties of
    the robot, simulator speed, the vision port.
    TODO: Also add functionality to change the field geometry properties
    TODO: GrSim ignores robotId when setting robot properties, resulting in all robots from the
    same team having the same properties. This is grSim's fault, so go make a pull request to
    grSim that fixes this faulty behavior. */
class ConfigurationCommand {
   public:
    // VelocityInRolling transforms the velocities into spinning engergy.
    // ByForce will set velocities accordingly to make sure the ball ends
    // up at the specified coordinates.
    void setBallLocation(float x, float y, float z, float xVelocity, float yVelocity, float zVelocity, bool velocityInRolling, bool teleportSafely, bool byForce);
    // Orientation is a global rotation relative to the field.
    // shouldBePresentOnField will make a robot (dis)appear accordingly.
    void addRobotLocation(int id, bool isFromTeamYellow, float x, float y, float xVelocity, float yVelocity, float angularVelocity, float orientation, bool shouldBePresentOnField,
                          bool byForce);
    void setSimulationSpeed(float speed);
    void addRobotSpecs(int id, bool isFromTeamYellow, RobotProperties& robotProperties);
    void setVisionPort(int port);

    proto::simulation::SimulatorCommand& getPacket();

   private:
    proto::simulation::SimulatorCommand configurationCommand;
};
}  // namespace rtt::robothub::simulation
