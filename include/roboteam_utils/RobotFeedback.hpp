#pragma once

#include <vector>

#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Teams.hpp>

namespace rtt {

/* The sources that can provide robot feedback */
enum class RobotFeedbackSource {
    SIMULATOR, BASESTATION
};

/* This type represents the feedback that a robot can give */
typedef struct RobotFeedback {
    int id = 0;                         // ID of the robot
    bool hasBall = false;               // The robot has the ball
    float ballPosition = 0.0f;          // [-0.5, 0.5] Position of the ball relative to the sensor
    bool ballSensorIsWorking = false;   // The ball sensor is working
    Vector2 velocity;                   // Estimated velocity according to the robot
    Angle angle;                        // Estimated angle according to the robot
    bool xSensIsCalibrated = false;     // The xSens is calibrated
    bool capacitorIsCharged = false;    // The capacitor is charged
    int wheelLocked = 0;                // Indicates if a wheel is locked. One bit per wheel
    int wheelBraking = 0;               // Indicates if a wheel is slipping. One bit per wheel
    float batteryLevel = 0;             // Battery level // TODO: Decide if this is in volts or percentage
    int signalStrength = 0;             // The signal strength of the robot

    bool operator== (const RobotFeedback& other) const;
} RobotFeedback;

/* This is the data object that is published by RobotHub */
typedef struct RobotsFeedback {
    Team team = Team::YELLOW;
    RobotFeedbackSource source = RobotFeedbackSource::SIMULATOR;
    std::vector<RobotFeedback> feedback;

    bool operator== (const RobotsFeedback& other) const;
} RobotsFeedback;

} // namespace rtt