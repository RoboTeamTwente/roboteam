#include "roboteam_robothub/packing.h"

#include <iostream>

#include <gtest/gtest.h>

TEST(PackingTestSuite, simplePacking) {
    // // Positive bounds checking
    // {
        // auto result = rtt::createRobotPacket(16, 8191, 511, true, 2047, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, 8192, 511, true, 2047, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, 8191, 512, true, 2047, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, 8191, 511, true, 2048, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, 8191, 511, true, 2047, 255, true, true, true, true, 8);
        // ASSERT_FALSE(!!result);
    // }

    // // Negative bounds
    // {
        // auto result = rtt::createRobotPacket(-1, 8191, 511, true, 2047, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, -1, 511, true, 2047, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, 8191, -1, true, 2047, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, 8191, 511, true, -1, 255, true, true, true, true, 7);
        // ASSERT_FALSE(!!result);
    // }

    // {
        // auto result = rtt::createRobotPacket(15, 8191, 511, true, 2047, 255, true, true, true, true, -1);
        // ASSERT_FALSE(!!result);
    // }

    // Handcrafted result checking
    {
        auto result = rtt::createRobotPacket(15, 8191, 511, true, 2047, 255, true, true, true, true, true, 7, 8191, 511, true, 2047);
        std::string correctResult = "11111111\n" // Everything at the highest value
                                    "11111111\n"
                                    "11111111\n"
                                    "11001111\n"
                                    "11111111\n"
                                    "11111111\n"
                                    "11111111\n"
                                    "11111111\n"
                                    "11111111\n"
                                    "11111111\n"
                                    "11111111\n"
                                    "11000000\n";
        ASSERT_TRUE(!!result);
        if (result) {
            ASSERT_EQ(correctResult, rtt::byteArrayToString<12>(*result));
        }
    }

    // {
        // auto result = rtt::createRobotPacket(0, 0, 0, false, 0, 0, false, false, false, false, 0);
        // std::string correctResult = "00000000\n" // Everything at the lowest value
                                    // "00000000\n"
                                    // "00000000\n"
                                    // "00000000\n"
                                    // "00000000\n"
                                    // "00000000\n"
                                    // "00000000\n";
        // ASSERT_TRUE(!!result);
        // if (result) {
            // ASSERT_EQ(correctResult, rtt::byteArrayToString<7>(*result));
        // }
    // }

    // {
        // std::string correctResult = "10100010\n"  // a: 12              1100
                                    // "11100110\n"  // b: 1484            0 0101 1100 1100
                                    // "01110001\n"  // 
                                    // "01000101\n"  // c: 453, d: false   1 1100 0101, 1
                                    // "10010111\n"  // e: 1431            101 1001 0111  
                                    // "00001100\n"  // f: 12              0000 1100
                                    // "01010110\n"; // g: true, h: false, i: true, j: false, k: 6
                                                  // //                    1, 0, 1, 0, 110

        // auto result = rtt::createRobotPacket(10, 1484, 453, false, 1431, 12, true, false, true, false, 6);
        // ASSERT_TRUE(!!result);
        // if (result) {
            // ASSERT_EQ(correctResult, rtt::byteArrayToString<7>(*result));
        // }
    // }

    // {
        // std::string correctResult = "11100001\n"  // a: 14              1110
                                    // "00010100\n"  // b: 553             0 0010 0010 1001
                                    // "10011010\n"  // 
                                    // "10001011\n"  // c: 106, d: true    0 0110 1010, 1
                                    // "11001110\n"  // e: 974             011 1100 1110
                                    // "01001110\n"  // f: 78              0100 1110
                                    // "00110111\n"; // g: false, h: true, i: true, j: false, k: 7
                                                  // //                    0, 1, 1, 0, 111

        // auto result = rtt::createRobotPacket(14, 553, 106, true, 974, 78, false, true, true, false, 7);
        // ASSERT_TRUE(!!result);
        // if (result) {
            // ASSERT_EQ(correctResult, rtt::byteArrayToString<7>(*result));
        // }
    // }
}

