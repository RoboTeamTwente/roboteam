//
// Created by rolf on 19-08-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/Time.h>

TEST(TimeTests, constructors) {
    Time empty;
    EXPECT_EQ(empty.asNanoSeconds(), 0);
    empty = Time::now();
    EXPECT_NE(empty.asNanoSeconds(), 0);
    Time after = empty.timeSince();
    Time before = empty.timeTo();
    EXPECT_GE(after.asNanoSeconds(), 0);
    EXPECT_LE(before.asNanoSeconds(), 0);
    long nanosecs = 1037450000;
    Time time(nanosecs);
    EXPECT_EQ(time.asNanoSeconds(), nanosecs);
}

TEST(TimeTests, conversions) {
    long nanosecs = 1037450000;
    Time time(nanosecs);
    EXPECT_DOUBLE_EQ(time.asSeconds(), 1.03745);
    EXPECT_EQ(time.asIntegerSeconds(), 1);
    EXPECT_EQ(time.asNanoSeconds(), nanosecs);
    EXPECT_EQ(time.asMicroSeconds(), 1037450);
    EXPECT_DOUBLE_EQ(time.asMilliSeconds(), 1037.45);
    EXPECT_EQ(time.asIntegerMilliSeconds(), 1037);
}
TEST(TimeTests, operators) {
    long a = 547289, b = 17438;
    Time timea(a), timeb(b);
    Time bminusa = timeb - timea;
    Time aminusb = timea - timeb;
    EXPECT_EQ(bminusa.asNanoSeconds(), b - a);
    EXPECT_EQ(aminusb.asNanoSeconds(), a - b);
    EXPECT_EQ((timea + timeb).asNanoSeconds(), a + b);

    Time timec = timea += timeb;
    EXPECT_EQ(timec.asNanoSeconds(), a + b);
    EXPECT_EQ(timea.asNanoSeconds(), a + b);
    EXPECT_EQ(timeb.asNanoSeconds(), b);

    Time timed = timea -= timeb;
    EXPECT_EQ(timed.asNanoSeconds(), a);
    EXPECT_EQ(timea.asNanoSeconds(), a);
    EXPECT_EQ(timeb.asNanoSeconds(), b);
}
TEST(TimeTests, booleans) {
    long nanosecs = 1037450000;
    Time time(nanosecs), time2(nanosecs);
    Time time3((long)1037450001);
    EXPECT_TRUE(time == time2);
    EXPECT_FALSE(time != time2);

    EXPECT_TRUE(time != time3);
    EXPECT_TRUE(time3 != time);
    EXPECT_FALSE(time == time3);
    EXPECT_FALSE(time3 == time);

    EXPECT_TRUE(time3 > time);
    EXPECT_FALSE(time > time3);
    EXPECT_TRUE(time3 >= time);
    EXPECT_TRUE(time2 >= time);
    EXPECT_FALSE(time >= time3);
    EXPECT_TRUE(time >= time2);

    EXPECT_FALSE(time3 < time);
    EXPECT_TRUE(time < time3);
    EXPECT_FALSE(time3 <= time);
    EXPECT_TRUE(time2 <= time);
    EXPECT_TRUE(time <= time3);
    EXPECT_TRUE(time <= time2);
}
TEST(TimeTests, doubleConstructor) {
    double duration = 3.482930;
    Time time(duration);
    EXPECT_DOUBLE_EQ(time.asSeconds(), duration);
}

TEST(TimeTests, minusBug) {
    Time one(1.01);
    Time two(2.01);
    Time three = two - one;
    EXPECT_DOUBLE_EQ((two - one).asSeconds(), 1.0);
    EXPECT_DOUBLE_EQ(three.asSeconds(), 1.0);
}