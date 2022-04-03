#pragma once

#include <roboteam_utils/Teams.hpp>
#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>

namespace rtt {

/* This struct represents all the info we can have about a robot */
typedef struct Robot {
    int id = 0.0;                       // [0, 15] The ID of the robot
    Team team = Team::YELLOW;           // [YELLOW, BLUE] The team of the robot
    Vector2 position;                   // (m) The position of the robot on the field
    Vector2 velocity;                   // (m/s) The velocity of the robot
    Angle angle;                        // (rad) The current angle of the robot
    double angularVelocity = 0.0;       // (rad/s) The current angular velocity

    bool ballSensorSeesBall = false;    // Indicates if the ball sensor sees the ball
    bool ballSensorIsWorking = false;   // Indicates if the ball sensor is working
    double ballPositionOnSensor = 0.0;  // [-0.5, 0.5] The position of the ball on the sensor

    double dribblerSpeed = 0.0;         // (rad/s) The current speed of the dribbler

    bool xSensIsCalibrated = false;     // Indicates if the xSens is calibrated
    bool capacitorIsCharged = false;    // Indicates if the capacitor is charged
    double signalStrength = 0.0;        // The signal strength of the robot
    double batteryLevel = 0.0;          // The battery level of the robot // TODO: Define units

    double radius = 0.09;               // (m) The radius of the robot
    double height = 0.15;               // (m) The height of the robot
    double frontWidth = 0.13;           // (m) The width of the front assembly
    double dribblerWidth = 0.10;        // (m) The width of the dribbler
    double capAngleOffset = 0.0;        // (rad) The difference between the angle of the cap vs the angle of the robot

    bool operator== (const Robot& other) const;
} Robot;

} // namespace rtt