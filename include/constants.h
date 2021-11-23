#pragma once

// define the amount of robots and the range of their IDs
constexpr int MAX_AMOUNT_OF_ROBOTS = 16;
constexpr int BASESTATION_VENDOR_ID = 0x0483;
constexpr int BASESTATION_PRODUCT_ID = 0x5740;

// 4096 bytes should be enough to receive any packet from the basestation.
constexpr int USB_BUFFER_SIZE_RECEIVE = 4096;
constexpr int SLEEP_NO_BASESTATION_MS = 1000;
constexpr int THREAD_READ_TIMEOUT_MS = 100;

// Simulation constants
constexpr int DEFAULT_GRSIM_CONFIGURATION_PORT = 10300;
constexpr int DEFAULT_GRSIM_CONTROL_PORT_BLUE = 10301;
constexpr int DEFAULT_GRSIM_CONTROL_PORT_YELLOW = 10302;

constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL = 30011;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL = 30012;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION = 30013;

constexpr float DEFAULT_KICK_SPEED = 5.0f;
constexpr float DEFAULT_CHIPPER_ANGLE = 45.0f;
constexpr float MAX_DRIBBLER_SPEED = 1021.0f;

constexpr int LISTEN_THREAD_COOLDOWN_MS = 10;  // Small cooldown in thread between checking for new messsages

// TODO: Beun constant, no idea which value this should be
constexpr int BATTERY_LOW_LEVEL = 1;
