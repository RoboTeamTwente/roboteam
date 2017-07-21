#include "roboteam_utils/Optimization.h"
#include <gtest/gtest.h>
#include <cstdio>

namespace rtt {
    
TEST(OptimizationTests, linear) {
    auto func = [](double x) { double t = x-4; return -(t*t); };
    // func is optimal at x=4
    ASSERT_DOUBLE_EQ(4.0, optimizeLinear(0, 10, 5, 50, func));
}
    
}