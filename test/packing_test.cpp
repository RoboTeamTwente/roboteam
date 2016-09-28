#include "roboteam_robothub/packing.h"

#include <iostream>

#include <gtest/gtest.h>

TEST(PackingTestSuite, simplePacking) {
    // Positive bounds checking
    {
        auto result = rtt::createRobotPacket(16, 4095, 511, true, 2047, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, 4096, 511, true, 2047, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, 4095, 512, true, 2047, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, 4095, 511, true, 2048, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, 4095, 511, true, 2047, 255, true, true, true, true, 8);
        ASSERT_FALSE(!!result);
    }

    // Negative bounds
    {
        auto result = rtt::createRobotPacket(-1, 4095, 511, true, 2047, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, -1, 511, true, 2047, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, 4095, -1, true, 2047, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, 4095, 511, true, -1, 255, true, true, true, true, 7);
        ASSERT_FALSE(!!result);
    }

    {
        auto result = rtt::createRobotPacket(15, 4095, 511, true, 2047, 255, true, true, true, true, -1);
        ASSERT_FALSE(!!result);
    }

    // Handcrafted result checking
    {
        auto result = rtt::createRobotPacket(15, 4095, 511, true, 2047, 255, true, true, true, true, 7);
        std::string correctResult = "11111111\n" // Everything at the highest value
                                    "11111111\n"
                                    "11111111\n"
                                    "00011111\n"
                                    "11111111\n"
                                    "11111111\n"
                                    "01111111\n";
        ASSERT_TRUE(!!result);
        if (result) {
            ASSERT_EQ(correctResult, rtt::byteArrayToString<7>(*result));
        }
    }

    {
        auto result = rtt::createRobotPacket(0, 0, 0, false, 0, 0, false, false, false, false, 0);
        std::string correctResult = "00000000\n" // Everything at the lowest value
                                    "00000000\n"
                                    "00000000\n"
                                    "00000000\n"
                                    "00000000\n"
                                    "00000000\n"
                                    "00000000\n";
        ASSERT_TRUE(!!result);
        if (result) {
            ASSERT_EQ(correctResult, rtt::byteArrayToString<7>(*result));
        }
    }

    {
        std::string correctResult = "10100101\n"  // a: 12
                                    "11001100\n"  // b: 1484
                                    "11100010\n"  // 
                                    "00010101\n"  // c: 453, d: false
                                    "10010111\n"  // e: 1431
                                    "00001100\n"  // f: 12
                                    "01010110\n"; // g: true, h: false, i: true, j: false, k: 6

        auto result = rtt::createRobotPacket(10, 1484, 453, false, 1431, 12, true, false, true, false, 6);
        ASSERT_TRUE(!!result);
        if (result) {
            ASSERT_EQ(correctResult, rtt::byteArrayToString<7>(*result));
        }
    }

    {
        std::string correctResult = "11100010\n"  // a: 14
                                    "00101001\n"  // b: 553
                                    "00110101\n"  // 
                                    "00001011\n"  // c: 106, d: true
                                    "11001110\n"  // e: 974
                                    "01001110\n"  // f: 78
                                    "00110111\n"; // g: false, h: true, i: true, j: false, k: 7

        auto result = rtt::createRobotPacket(14, 553, 106, true, 974, 78, false, true, true, false, 7);
        ASSERT_TRUE(!!result);
        if (result) {
            ASSERT_EQ(correctResult, rtt::byteArrayToString<7>(*result));
        }
    }
}

