//
// Created by rolf on 19-08-20.
//

#include <gtest/gtest.h>
#include <roboteam_utils/Random.h>
#include <roboteam_utils/Time.h>

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
        successes += (abs(sum) <= 2 * sigma);  // 95 % confidence interval.
    }
    EXPECT_GE(successes, 90);  // Leave some width for 95% confidence interval.
    // The actual value is 94, which is a good indication everything works as intended.
}