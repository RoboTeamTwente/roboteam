#pragma once

#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Vector2.h>

#include <roboteam_utils/Teams.hpp>

namespace rtt {

/* This struct represents all the info we can have about a robot */
typedef struct Robot {
    int id = 0;                    // [0, 15] The ID of the robot
    Team team = Team::YELLOW;      // [YELLOW, BLUE] The team of the robot
    Vector2 position;              // (m) The position of the robot on the field
    Vector2 velocity;              // (m/s) The velocity of the robot
    Angle yaw;                     // (rad) The current yaw of the robot
    double angularVelocity = 0.0;  // (rad/s) The current angular velocity

    bool ballSensorSeesBall = false;   // Indicates if the ball sensor sees the ball
    bool ballSensorIsWorking = false;  // Indicates if the ball sensor is working
    bool dribblerSeesBall = false;     // Indicates if the dribbler sees the ball

    double dribblerOn = 0.0;  // (rad/s) The current speed of the dribbler

    bool xSensIsCalibrated = false;   // Indicates if the xSens is calibrated
    bool capacitorIsCharged = false;  // Indicates if the capacitor is charged
    double batteryLevel = 0.0;        // The battery level of the robot // TODO: Define units

    double radius = 0.09;         // (m) The radius of the robot
    double height = 0.15;         // (m) The height of the robot
    double frontWidth = 0.13;     // (m) The width of the front assembly
    double dribblerWidth = 0.10;  // (m) The width of the dribbler
    Angle capOffset;              // (rad) The difference between the angle of the cap vs the yaw of the robot

    bool operator==(const Robot& other) const;
} Robot;

}  // namespace rtt