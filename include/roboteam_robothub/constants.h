//
// Created by mrlukasbos on 12-3-19.
//

#ifndef ROBOTEAM_CONSTANTS_H_ROBOTHUB
#define ROBOTEAM_CONSTANTS_H_ROBOTHUB

// define the amount of robots and the range of their IDs
/**
 * C++11 / 14 introduce constexpr, a replacement for macros
 */
constexpr int MAX_AMOUNT_OF_ROBOTS = 16;
constexpr int MIN_ROBOT_ID = 0;
constexpr int MAX_ROBOT_ID = 15;
constexpr int ROBOTHUB_TICK_RATE = 60;
constexpr int BASESTATION_VENDOR_ID = 0x0483;
constexpr int BASESTATION_PRODUCT_ID = 0x5740;

#endif  // ROBOTEAM_CONSTANTS_H_ROBOTHUB
