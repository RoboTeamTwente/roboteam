#pragma once

#include <roboteam_proto/ssl_simulation_robot_control.pb.h>

namespace rtt::robothub::simulation {
/*  This class contains the information for moving robots.
    It immediately stores the given information into a packet that can be sent to the simulator.
    Robots can be controlled by either wheel speeds, local velocities or global velocties.
    Local velocities are speeds like forward and sideway speeds, while global velocties are
    speeds relative to the field. */
class RobotControlCommand {
   public:
    void addRobotControlWithWheelSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float frontRightWheelVelocity, float backRightWheelVelocity,
                                        float backLeftWheelVelocity, float frontLeftWheelVelocity);
    void addRobotControlWithLocalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float forwardVelocity, float leftVelocity, float angularVelocity);
    void addRobotControlWithGlobalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float xVelocity, float yVelocity, float angularVelocity);
    proto::sim::RobotControl& getPacket();

   private:
    proto::sim::RobotControl controlCommand;
};
}  // namespace rtt::robothub::simulation