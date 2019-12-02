#include "../include/roboteam_utils/Optimization.h"
#include <cmath>
#include <random>

namespace rtt {

    double clamp(double d, double min, double max) {
        if (d < min) return min;
        if (d > max) return max;
        return d;
    }

    double
    optimizeLinear(double from, double to, double step, int refinement, std::function<double(double)> scoreFunc) {
        double candidate = NAN;
        double previous = from + (to - from) / 2;
        double lastScore = scoreFunc(previous);
        int refCount = 0;
        int movementSign = 1;
        while (true) {
            candidate = clamp(previous + movementSign * step, from, to);
            double score = scoreFunc(candidate);
            if (score < lastScore) {
                if (++refCount < refinement) {
                    step /= 2.0;
                    movementSign *= -1;
                } else {
                    return previous;
                }
            }
            previous = candidate;
            lastScore = score;
        }
    }

    Vector2 optimizeVector(
            const Vector2 &center,
            double minXDev,
            double maxXDev,
            double minYDev,
            double maxYDev,
            double step,
            int refinement,
            std::function<double(const Vector2 &)> scoreFunc) {
        return Vector2(); // TODO
    }

    Vector2 sampleForVector(
            const Vector2 &center,
            double minXDev,
            double maxXDev,
            double minYDev,
            double maxYDev,
            int samples,
            std::function<double(const Vector2 &)> scoreFunc) {
        static std::mt19937 rnd;
        double bestScore = NAN;
        double minX = center.x + minXDev;
        double minY = center.y + minYDev;
        double maxX = center.x + maxXDev;
        double maxY = center.y + maxYDev;

        Vector2 bestVec;
        for (int i = 0; i < samples; i++) {
            double x = minX + (maxX - minX) * rnd();
            double y = minY + (maxY - minY) * rnd();
            Vector2 vec(x, y);
            double score = scoreFunc(vec);
            if (score > bestScore) {
                bestScore = score;
                bestVec = vec;
            }
        }
        return bestVec;
    }

}
