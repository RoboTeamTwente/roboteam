//
// Created by rolf on 19-08-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>
#include <roboteam_utils/Time.h>

#include <iterator>

TEST(Random, constructors) {
    long nanoSeconds = 42;
    Time time(nanoSeconds);
    Random random1(time);
    Random random2(nanoSeconds);
    EXPECT_DOUBLE_EQ(random1.getGaussian(), random2.getGaussian());  // both constructors should initialize with the same random number
}
TEST(Random, multiple) {
    Random gen(42);
    EXPECT_NE(gen.getGaussian(), gen.getGaussian());  // consecutive numbers should not be the same after initialization
}
TEST(Random, uniform) {
    Random gen(42);
    for (int kI = 0; kI < 100; ++kI) {
        double x = gen.getUniform();
        EXPECT_GE(x, 0.0);
        EXPECT_LE(x, 1.0);
    }
}
TEST(Random, instance) {
    Random control(42);
    Random test(42);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    EXPECT_DOUBLE_EQ(distribution(test.instance()), control.getUniform());
}
TEST(Random, gaussian) {
    int successes = 0;
    for (int i = 0; i < 100; ++i) {
        Random gen(i);
        // Use central limit theorem as test
        int n = 10000;
        double sum = 0;
        for (int kI = 0; kI < n; ++kI) {
            sum += gen.getGaussian();
        }
        sum /= (double)n;
        double sigma = 1 / sqrt(n);
        successes += (fabs(sum) <= 2 * sigma);  // 95 % confidence interval.
    }
    EXPECT_GE(successes, 90);  // Leave some width for 95% confidence interval.
    // The actual value is 94, which is a good indication everything works as intended.
}

TEST(Random, simpleIntRange) {
    constexpr int RANGE = 20;
    for (int i = 0; i < 50; i++) {
        int base = SimpleRandom::getInt();
        int low = base - RANGE;
        int high = base + RANGE;

        for (int j = 0; j < 100; j++) {
            int test = SimpleRandom::getInt(low, high);
            EXPECT_GE(test, low);
            EXPECT_LE(test, high);
        }
    }
}

TEST(Random, simpleInt) {
    bool hasNegative = false;
    bool hasPositive = false;
    for (int i = 0; i < 100; i++) {
        int test = SimpleRandom::getInt();
        if (test > 0) hasPositive = true;
        if (test < 0) hasNegative = true;
    }
    EXPECT_TRUE(hasPositive);
    EXPECT_TRUE(hasNegative);
}

TEST(Random, simpleLongRange) {
    constexpr long RANGE = 20;
    for (int i = 0; i < 50; i++) {
        long base = SimpleRandom::getInt();
        long low = base - RANGE;
        long high = base + RANGE;

        for (int j = 0; j < 100; j++) {
            long test = SimpleRandom::getInt(low, high);
            EXPECT_GE(test, low);
            EXPECT_LE(test, high);
        }
    }
}

TEST(Random, simpleLong) {
    bool hasNegative = false;
    bool hasPositive = false;
    for (int i = 0; i < 100; i++) {
        long test = SimpleRandom::getInt();
        if (test > 0) hasPositive = true;
        if (test < 0) hasNegative = true;
    }
    EXPECT_TRUE(hasPositive);
    EXPECT_TRUE(hasNegative);
}

TEST(Random, simpleDoubleRange) {
    for (int i = 0; i < 50; i++) {
        double base = SimpleRandom::getDouble();
        double range = std::fabs(SimpleRandom::getDouble());
        double low = base - range;
        double high = base + range;

        for (int j = 0; j < 100; j++) {
            double test = SimpleRandom::getDouble(low, high);
            EXPECT_GE(test, low);
            EXPECT_LE(test, high);
        }
    }
}

TEST(Random, simpeDouble) {
    bool hasNegative = false;
    bool hasPositive = false;
    for (int i = 0; i < 100; i++) {
        double test = SimpleRandom::getDouble();
        if (test > 0.0) hasPositive = true;
        if (test < 0.0) hasNegative = true;
    }
    EXPECT_TRUE(hasPositive);
    EXPECT_TRUE(hasNegative);
}

TEST(Random, simpleBool) {
    bool hasFalse = false;
    bool hasTrue = false;
    for (int i = 0; i < 50; i++) {
        bool test = SimpleRandom::getBool();
        if (test) {
            hasTrue = true;
        } else {
            hasFalse = true;
        }
    }
    EXPECT_TRUE(hasFalse);
    EXPECT_TRUE(hasTrue);
}

TEST(Random, simpleRandElementEmpty) {
    std::vector<int> empty = {};
    auto randEmptyElement = SimpleRandom::getRandomElement(empty.begin(), empty.end());
    EXPECT_EQ(randEmptyElement, empty.end());
}

TEST(Random, simpleRandElementSelection) {
    std::vector<int> vec = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    for (int i = 0; i < 100; i++) {
        auto randomIt = SimpleRandom::getRandomElement(vec.begin(), vec.end());
        EXPECT_NE(randomIt, vec.end());
        EXPECT_GE(*randomIt, 0);
        EXPECT_LE(*randomIt, 9);
    }
}
