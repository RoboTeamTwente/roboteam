#pragma once

#include "Vector2.h"
#include <functional>

namespace rtt {
    
/*
 * Finds the value in a given range which produces the maximum result in a given function.
 * A greater initial step with higher refinement lowers the risk of finding a local maximum, but 
 * it's also slower.
 */
double optimizeLinear(
    double from,                                // The low end of the range
    double to,                                  // The high end of the range
    double step,                                // The initial step size
    int refinement,                             // The amount of times to refine (=half) the step size before returning the best result
    std::function<double(double)> scoreFunc     // The scoring function. Higher values are better.
);

Vector2 optimizeVector(
    const Vector2& center,
    double minXDev,
    double maxXDev,
    double minYDev,
    double maxYDev,
    double step,
    int refinement,
    std::function<double(const Vector2&)> scoreFunc
);

Vector2 sampleForVector(
    const Vector2& center,
    double minXDev,
    double maxXDev,
    double minYDev,
    double maxYDev,
    int samples,
    std::function<double(const Vector2&)> scoreFunc
);

}